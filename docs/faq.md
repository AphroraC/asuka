# FAQ

### RViz 里没有 TF？

TF（`world -> imu`）所有权规则：`config_extensions.json` 中 `publish_odometry_imu=true`
时，高频 TF 由 `imu_prediction` 负责，`rviz_viewer` 让位；若 `imu_prediction` 未加载而该
开关仍为 true，则无人发布 TF。两者配置需匹配，或将 `publish_tf` 与流开关显式对齐。

### 切换 robosense 雷达要注意什么？

两处必须一致：`config_ros.json` 的 `lidar_type`，以及 `config_sensor.json` 中只保留
`robosense` 块（livox 块注释掉）。sensor 块选择按 `has("livox") -> has("robosense")`
优先级进行，与 `lidar_type` 相互独立。

### 为什么编译比普通 ROS 包慢？

CMake 全局定义了 `PCL_NO_PRECOMPILE`（自定义点类型所需），PCL 模板算法在每个使用
单元就地实例化。属预期行为，见[安装](installation.md)。

### 为什么不编译后端？

`BUILD_OPTIMIZATION_MODULES` 默认 `OFF`（不依赖 GTSAM）。需要
`optimization_loam` / `optimization_miao` 时传
`-DBUILD_OPTIMIZATION_MODULES=ON`，见[安装](installation.md)。

### 同一进程能加载多个 asuka nodelet 吗？

不建议。`GlobalConfig` 是非线程安全的进程级单例（重建时 delete+new），多实例会互相
覆盖配置。每进程一个 `AsukaROS`。

### 数据时间回绕（rosbag 循环回放）会怎样？

`AsukaROS` 检测到回绕只告警；batchlio 会丢弃乱序样本，fastlio/smallpointlio 不做处理，
基类的 `clear_buffers()` 目前框架未调用。长时间循环回放前请确认所用算法的回绕行为，
见[里程计插件](odometry.md)。

### 如何添加新算法 / 新扩展？

分别见 [里程计插件](odometry.md) 与
[扩展开发](extend.md)。
