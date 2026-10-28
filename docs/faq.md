# FAQ

### RViz 里没有 TF？

TF（`world -> imu`）所有权规则：`config_extensions.json` 中 `publish_odometry_imu=true`
时，高频 TF 由 `imu_prediction` 负责，`rviz_viewer` 让位；若 `imu_prediction` 未加载而该
开关仍为 true，则无人发布 TF。两者配置需匹配，或将 `publish_tf` 与流开关显式对齐。

### 切换雷达类型（livox / livox_custom / robosense）要注意什么？

两处要保持一致：`config_ros.json` 的 `lidar_type` 决定消息反序列化路径；外参取自
**所加载前端自己的 `config_odometry` 文件的 `sensor` 块**（每个算法对齐其上游基准外参，
不再有共享的 config_sensor.json）。切雷达时请核对该前端配置的外参是否对应当前设备。
三前端均已实现 `livox_custom` 处理（fastlivo 为上游 avia_handler 直移植，其余两前端为
消息格式适配），无需按前端挑选雷达格式。另外 `RobosensePoint` 的逐点时间戳
契约是纳秒（见 [点云格式](point_type.md)），使用按秒发布的驱动需在录制端转换。

### 为什么编译比普通 ROS 包慢？

CMake 全局定义了 `PCL_NO_PRECOMPILE`（自定义点类型所需），PCL 模板算法在每个使用
单元就地实例化。属预期行为，见[安装](installation.md)。

### LIVO 前端喂无图 bag 为什么变成 0.2x 慢速空转？

`slam_mode: LIVO` 的前端（fastlivo）同步器以图像时刻锚定测量窗：bag 里没有
图像话题时同步器永久等待、内部雷达缓冲持续积压，rosbag 回放的背压门（`workload > 10`
即节流）把喂入压到每 500 ms 一帧——表现为 ~0.2x 回放速率且 CPU 近零。处理：无图素材把
fastlivo 的 `slam_mode` 改为 `ONLY_LIO`；配置检查见
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

统一评测方法：`asuka_rosbag` 全速回放（带背压），以 fastlio 轨迹为参考按时间戳插值
计算位置 RMSE。下表为 2026-10-23 同法重测结果；两前端终点闭环
漂移 0.561-0.567 m 相互吻合。
fastlivo 为 LIVO 前端（评测素材与结果见 [里程计插件](odometry.md)），不参与本表：

| 前端 | RMSE vs fastlio | 备注 |
|---|---|---|
| fastlio | 参考 | 闭环终点漂移 0.567 m |
| superlio | 0.81 cm | 中位 0.65 cm / 最大 2.88 cm（2026-10-26 去畸变越界分支修复后；修复前 2.77 / 2.32 / 6.6 cm——见 [benchmark](benchmark.md) 注记） |

### 如何添加新算法 / 新扩展？

分别见 [里程计插件](odometry.md) 与
[扩展开发](extend.md)。
