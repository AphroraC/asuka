# FAQ

### RViz 里没有 TF？

TF（`world -> imu`）所有权规则：`config_extensions.json` 中 `publish_odometry_imu=true`
时，高频 TF 由 `imu_prediction` 负责，`rviz_viewer` 让位；若 `imu_prediction` 未加载而该
开关仍为 true，则无人发布 TF。两者配置需匹配，或将 `publish_tf` 与流开关显式对齐。

### 切换雷达类型（livox / livox_custom / robosense）要注意什么？

两处要保持一致：`config_ros.json` 的 `lidar_type` 决定消息反序列化路径；外参取自
**所加载前端自己的 `config_odometry` 文件的 `sensor` 块**（每个算法对齐其上游基准外参，
不再有共享的 config_sensor.json）。切雷达时请核对该前端配置的外参是否对应当前设备。
六前端均已实现 `livox_custom` 处理（fastlivo 为上游 avia_handler 直移植，其余为
消息格式适配），无需按前端挑选雷达格式。另外 `RobosensePoint` 的逐点时间戳
契约是纳秒（见 [点云格式](point_type.md)），使用按秒发布的驱动需在录制端转换。

### 为什么编译比普通 ROS 包慢？

CMake 全局定义了 `PCL_NO_PRECOMPILE`（自定义点类型所需），PCL 模板算法在每个使用
单元就地实例化。属预期行为，见[安装](installation.md)。

### 为什么不编译后端？

`BUILD_OPTIMIZATION_MODULES` 默认 `OFF`（不依赖 GTSAM）。需要
`optimization_loam` 时传
`-DBUILD_OPTIMIZATION_MODULES=ON`，见[安装](installation.md)。

### LIVO 前端喂无图 bag 为什么变成 0.2x 慢速空转？

`slam_mode: LIVO` 的前端（fastlivo / srlivo）同步器以图像时刻锚定测量窗：bag 里没有
图像话题时同步器永久等待、内部雷达缓冲持续积压，rosbag 回放的背压门（`workload > 10`
即节流）把喂入压到每 500 ms 一帧——表现为 ~0.2x 回放速率且 CPU 近零。处理：无图素材把
fastlivo 的 `slam_mode` 改为 `ONLY_LIO`（srlivo 无此模式，必须带图 bag）；配置检查见
[参数速查](parameters.md) 的 `config_odometry × lidar_type` 组合告警说明。

### 同一进程能加载多个 asuka nodelet 吗？

不能。`GlobalConfig` 是进程级单例（函数内静态、首次调用定型、线程安全），回调槽等
全局静态对象亦每进程一份；`AsukaROS` 构造函数带进程级单实例守卫，同进程构造第二个
实例（如同一 manager 加载两个 asuka nodelet）会直接抛异常。

### 数据时间回绕（rosbag 循环回放）会怎样？

回环检测统一在 `AsukaROS::handle_loop_back` 完成：检测到时间回退仅告警，不做丢弃、
不清缓冲，数据按新时间线继续流入。各算法内部不再有第二道回环处理。长时间循环回放
前请自行确认所用算法对乱序数据的容忍度，见[里程计插件](odometry.md)。

### 各前端在 liantiao.bag（Mid-360，116 s，~101 m 闭环步行）上的已知表现

统一评测方法：`asuka_rosbag` 全速回放（带背压），以 fastlio 轨迹为参考计算 RMS。
结论速览（fastlivo 为 LIVO 前端，评测素材与结果见 [里程计插件](odometry.md)，
不参与本表）：

| 前端 | RMS vs fastlio | 备注 |
|---|---|---|
| fastlio | 参考 | 闭环终点残差 < 0.1 m |
| superlio | 0.03 m | 闭环 |
| batchlio | 8.90 m（修复前基线，input 模式） | 已知问题 1 的两个移植缺陷已于 2026-09-26 修复（饱和门单位 + VoxelGrid 字段清零，后者由 `PointXYZITRGB`/`curvature` 改名根治）；output/input 模式现均稳定闭环（终点 ~0.56 m），RMS 待重测 |

  srlivo 未参评（需要图像流，liantiao 无图；验证素材见[里程计插件](odometry.md)）；
  zlio 亦未参评（2026-10 迁移，验证记录见[里程计插件](odometry.md)的 zlio 节）。

1. **batchlio**（2026-09-26 已修复，见下）：`use_imu_as_input=false`（output 模式，
   上游默认）曾在本包上运动开始后发散（速度线性增长至 ~76 m/s，~38% 处飞出至
   2.4 km 外），过程模型 / 量测模型 / esekfom 引擎与上游逐字节一致。**根因（与上游
   A/B 实验定位）是两个移植层缺陷叠加**：① 饱和门单位错配——IMU 量测更新的加速度
   饱和检查以 SI（m/s²，acc_scale 缩放后）进料对比 g 单位的 `satu_acc: 3.0`，静止
   重力即触发，acc/ba 的 IMU 锚定从首帧起被清零（修复：阈值乘 `acc_norm` 化为物理
   单位不变式，上游 acc_norm=1.0 语义逐位不变）；② **pcl::VoxelGrid 丢失自定义
   `offset` 字段**——pcl::detail::Accumulators 只覆盖已知字段（xyz/normal/curvature/
   rgb/intensity/label），下采样后逐点时间全零，批窗口塌缩为 1，批内去畸变
   （eq. 3.44-3.47）从未执行（上游不受影响：其 PointXYZINormal 用 stock `curvature`
   携带时间，有专用累加器）。**最终修复**：点类型更名为 `PointXYZITRGB`，时间字段改用
   stock `curvature`（PCL 原生累加器，VoxelGrid 自动按体素平均），batchlio 恢复
   `pcl::VoxelGrid`，整类"自定义字段被滤镜清零"问题从机制上消除（ interim 的
   `voxel_downsample_with_time` 已删）。修复后 output 模式（已恢复为上游默认）
   在 liantiao.bag 闭环终点 0.567 m（上游参考 0.556 m），速度峰值 1.90 vs 2.03 m/s；
   input 模式同样受益（终点 0.556 m）。同批已把 `ivox_nearby_type` /
   `filter_size_map` / `point_filter_num` / `init_map_size` / `imu_meas_*_cov`
   对齐回 avia.yaml 基准。
### 如何添加新算法 / 新扩展？

分别见 [里程计插件](odometry.md) 与
[扩展开发](extend.md)。
