# Point cloud format

## PointXYZIOffset —— 32 字节紧凑点类型

全项目统一使用 `asuka::PointXYZIOffset`（`asuka/core/types.hpp`）替代
`pcl::PointXYZINormal`（48 B）：

```cpp
struct EIGEN_ALIGN16 PointXYZIOffset {
  PCL_ADD_POINT4D;      // x y z (+16 字节对齐垫)
  float intensity;      // 回波强度（透传给可视化 / 存图）
  float offset;         // 点时间相对扫描时间基准的毫秒偏移
};
static_assert(sizeof(PointXYZIOffset) == 32);
```

设计要点：

- 48 B 中 16 B 的 normal 字段全项目从未使用，去除后内存与带宽节省 33%；
- `PCL_ADD_POINT4D` 的对齐垫维持 16 字节对齐（Eigen/SIMD 友好）——这是 PCL 点类型的
  标准做法，5 个 float（20 B）按 16 B 对齐圆整为 32 B；
- 构造函数清零全部字段（含对齐垫），保证 `DefaultPointRepresentation` 整结构拷贝类
  算法（如 ICP）的确定性；
- CMake 全局定义 `PCL_NO_PRECOMPILE`，使 PCL 模板算法支持该自定义类型。

## offset 与帧级时间戳

- `PointT::offset` 存储点时刻相对**扫描时间基准**的毫秒数（float；
  扫描内相对时间下 float 的分辨率优于微秒，足够）；
- 扫描的时间基准由各算法的 `CloudPreprocess` 取扫描首点（或头文件）时间戳；
- 帧级时间戳记录在 `KeyFrame::stamp`，取首点或末点时间戳由具体算法决定：
  fastlio / batchlio / lightning 取扫描结束时刻，smallpointlio 取当前点更新时刻，
  superlio 取状态时间戳。

## intensity 的用途

intensity 的**值**不参与任何算法的估计数学，仅透传到 `KeyFrame::cloud_imu` 供可视化
（rviz_viewer）与存图使用；但 **intensity 字段本身**被 fastlio / batchlio 用作平面残差的
暂存槽（`normvec->intensity = 平面距离`，随后在观测模型中读回），属于 FAST-LIO2 家族的
实现惯例，请勿向该字段写入其他数据。`curvature` 的旧用途（时间偏移）已由 `offset` 字段
显式承载。

## 与 ROS 消息的边界

`sensor_msgs::PointCloud2` 在 `AsukaROS` 中按 `lidar_type` 反序列化为
`LivoxPoint` / `RobosensePoint`（两者均含 `double timestamp`），经预处理转为
`PointCloudT`；输出侧 rviz_viewer / 存图均转为 `pcl::PointXYZI`，48 B 内部布局不出进程。
