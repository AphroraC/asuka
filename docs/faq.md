# FAQ

### RViz 里没有 TF？

TF（`world -> imu`）所有权规则：`config_extensions.json` 中 `publish_odometry_imu=true`
时，高频 TF 由 `imu_prediction` 负责，`rviz_viewer` 让位；若 `imu_prediction` 未加载而该
开关仍为 true，则无人发布 TF。两者配置需匹配，或将 `publish_tf` 与流开关显式对齐。

### 切换 robosense 雷达要注意什么？

只需两步：`config_ros.json` 的 `lidar_type` 改为 `robosense`，并确认 `config_sensor.json`
中存在 `robosense` 块（含外参与噪声）。各算法经 `active_lidar_key()` 按 `lidar_type`
选择 sensor 块，块缺失会在启动期抛出明确异常；未启用的块保留在文件中不会生效。

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

回环检测统一在 `AsukaROS::handle_loop_back` 完成：检测到时间回退仅告警，不做丢弃、
不清缓冲，数据按新时间线继续流入。各算法内部不再有第二道回环处理。长时间循环回放
前请自行确认所用算法对乱序数据的容忍度，见[里程计插件](odometry.md)。

### 五种前端在 liantiao.bag（Mid-360，116 s，~101 m 闭环步行）上的已知表现

统一评测方法：`asuka_rosbag` 全速回放（带背压），以 fastlio 轨迹为参考计算 RMS。
结论速览：

| 前端 | RMS vs fastlio | 备注 |
|---|---|---|
| fastlio | 参考 | 闭环终点残差 < 0.1 m |
| smallpointlio | 3.27 m | 闭环 |
| superlio | 0.03 m | 闭环 |
| batchlio | 8.90 m | input 模式，见下方已知问题 1 |
| lightning | 3.29 m | 闭环，见下方已知问题 2 |

1. **batchlio**：`use_imu_as_input=false`（output 模式，上游默认）在本包上运动开始后
   出现恒定伪加速度发散（速度线性增长至 ~76 m/s）。过程模型 / 量测模型 / esekfom
   引擎经逐字节 diff 与上游一致；`acc_norm`、`satu_acc` 单位、`batch_deskew` 消融、
   初始重力对齐时序、`imu_meas_acc/omg_cov` 对齐至上游 0.01 等均已排除——发散源于
   移植层数据流编排（synchronize / imu_deque 填充 / 自定义窗口去畸变与迭代 H 的
   交互），待与上游在同一 bag 上 A/B 定位。当前配置已切换为
   `use_imu_as_input: true`（input 模式，见 config 内注释）。
2. **lightning**：曾有 `sync_packages` 空指针崩溃（已修）、轨迹低估（已修）与
   **导出地图退化**（已修）。轨迹低估根因是配置漂移：ROI 高度带 ±1.0 m
   （上游 0.5~10 m）丢弃了绝大部分场景结构、`plane_icp_weight` 1.0（上游 300.0）、
   `enable_icp_part` true（上游 false）、`ivox_grid_resolution` 0.2（上游 0.5）、
   `skip_lidar_num` 5（上游 0）。配置对齐上游后闭环收敛（RMS 3.29 m，与
   smallpointlio 相当）。导出地图退化为单帧残影的根因是 `make_kf` 让关键帧共享
   `scan_undistort` 成员缓冲——下一帧的 `clear()` 会清空所有已存关键帧的点云，
   现已改为关键帧持有私有拷贝（修复后地图 29 万点，结构与 fastlio 一致）。
3. **smallpointlio**：`save_map()` 返回空（small iVox 不支持全图导出），地图落盘
   对该前端不可用。

### 如何添加新算法 / 新扩展？

分别见 [里程计插件](odometry.md) 与
[扩展开发](extend.md)。
