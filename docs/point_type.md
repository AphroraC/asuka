# Point cloud format

## PointXYZITRGB —— 32 字节紧凑点类型

全项目统一使用 `asuka::PointXYZITRGB`（`asuka/core/types.hpp`）替代
`pcl::PointXYZINormal`（48 B）：

```cpp
struct EIGEN_ALIGN16 PointXYZITRGB {
  PCL_ADD_POINT4D;       // x y z (+16 字节对齐垫)
  float intensity;       // 回波强度（透传给可视化 / 存图）
  float curvature;       // 点时刻相对扫描时间基准的毫秒偏移（FAST-LIO 系惯例字段）
  PCL_ADD_RGB;           // rgb 联合体（LIVO 前端填色；LIO 前端恒 0）
};
static_assert(sizeof(PointXYZITRGB) == 32);
```

（历史沿革：原名 `PointXYZIOffset`，OpenCV 无条件化后与 `PointXYZIRGBOffset` 合流；
2026-09-26 起时间字段改用 stock `curvature` 并更名 `PointXYZITRGB`，见下。）

设计要点：

- 48 B 中 16 B 的 normal 字段全项目从未使用，去除后内存与带宽节省 33%；
- `PCL_ADD_POINT4D` 的对齐垫维持 16 字节对齐（Eigen/SIMD 友好）——这是 PCL 点类型的
  标准做法，5 个 float（20 B）按 16 B 对齐圆整为 32 B；
- 构造函数清零全部字段（含对齐垫），保证 `DefaultPointRepresentation` 整结构拷贝类
  算法（如 ICP）的确定性；
- CMake 全局定义 `PCL_NO_PRECOMPILE`，使 PCL 模板算法支持该自定义类型。

## curvature 承载点时刻（PCL 滤镜兼容性关键）

- `PointT::curvature` 存储点时刻相对**扫描时间基准**的毫秒数（float；
  扫描内相对时间下 float 的分辨率优于微秒，足够）；
- **为什么用 stock `curvature` 而不是自定义字段**：`pcl::detail::Accumulators` 仅为
  已知字段（xyz/normal/curvature/rgba/intensity/label）提供质心累加器，自定义字段在
  `pcl::VoxelGrid` 等经 `CentroidPoint` 的滤镜下会被**清零**（2026-09-26 batchlio
  发散问题的根因之一：下采样后逐点时间归零 → 批内去畸变静默失效）。`curvature` 命中
  `pcl::traits::has_curvature`（注册标签级判定，无需 normal 字段），VoxelGrid 对其
  **原生按体素平均**；容器内已实证 32 B 布局 + rgb + curvature 均值全部保留；
- 扫描的时间基准由各算法的 `CloudPreprocess` 取扫描首点（或头文件）时间戳；
- 帧级时间戳记录在 `KeyFrame::stamp`，契约统一定为**产生关键帧的传感器扫描的
  起始时刻**（见 `callbacks.hpp`）；载荷位姿通常是估计器当前状态（多指扫描末），
  消费者不得假设二者重合。

## intensity 与 rgb 的用途

intensity 的**值**不参与任何算法的估计数学，仅透传到 `KeyFrame::cloud_imu` 供可视化
（rviz_viewer）与存图使用；但 **intensity 字段本身**被 fastlio / batchlio 用作平面残差的
暂存槽（`normvec->intensity = 平面距离`，随后在观测模型中读回），属于 FAST-LIO2 家族的
实现惯例，请勿向该字段写入其他数据。

rgb 仅由 fastlivo / srlivo 在视觉更新时填真实颜色，LIO 前端恒 0。rviz 输出经
`DisplayPoint` 带 rgb 字段，地图 PCD 为 `x y z intensity rgb`。

## 与 ROS 消息的边界

`sensor_msgs::PointCloud2` 在 `AsukaROS` 中按 `lidar_type` 反序列化为
`LivoxPoint` / `RobosensePoint` / `LivoxCustomPoint`（均含纳秒级时间信息），经预处理转为
`PointCloudT`；输出侧 rviz_viewer 转为 `DisplayPoint`、存图转为 `PointXYZIRGB`，
32 B 内部布局不出进程。时间单位契约：
`RobosensePoint::timestamp` 为**纳秒**（已对录制的 robosense bag 实证；上游 FAST-LIO
按秒解释，录制端需转换），`LivoxCustomPoint::offset_time` 为驱动给出的纳秒偏移。
