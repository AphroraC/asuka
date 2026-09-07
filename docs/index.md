# Home

**asuka** 是一个稳健、精简、高效、可扩展的 LiDAR-Inertial SLAM 框架（ROS1 / catkin）。
它将多种 LiDAR-Inertial 里程计算法统一到同一套插件接口之下，通过全局回调插槽把前端状态
分享给运行时加载的扩展模块（后端位姿图优化、高频 IMU 预测、可视化等）。

## 特性

- ***算法即插件*** —— FAST-LIO2、Batch-LIO、Small Point-LIO、Super-LIO、Lightning 五种前端里程计
  各自编译为独立动态库，运行时经 `dlopen` 加载，修改 `config.json` 即可切换算法。
- ***稳健*** —— 统一的 `KeyFrame` 输出契约；回调插槽逐观察者隔离异常；每个算法独立的参数文件；
  框架层统一处理时间回绕检测、地图保存与优雅退出。
- ***精简高效*** —— 全项目使用 32 字节紧凑点类型 `asuka::PointXYZIOffset`（替代 48 B 的
  `pcl::PointXYZINormal`），无冗余字段；点内仅保留 xyz、intensity 与毫秒级时间偏移。
- ***可扩展*** —— 全局回调插槽（global callback slots）暴露 IMU 输入、帧输入、关键帧、
  IMU 预测里程计与后端优化里程计五类内部状态；扩展模块即共享库，无需修改框架代码。

## 总体架构

```
asuka_rosnode / asuka_rosbag / asuka_nodelet      <- ROS 入口（三选一）
        |  sensor_msgs::Imu / PointCloud2
        v
    AsukaROS                                       <- ROS 层：消息转换、acc_scale、
        |  asuka::ImuData / PointCloudT               时间偏移、点云预处理、扩展加载
        v
AsyncOdometryEstimation                            <- 异步执行器（前端 worker 线程）
        |
        v
asuka::OdometryEstimation   <== dlopen ==   libasuka_odometry_*.so
        |                                             (fastlio / batchlio / smallpointlio /
        |  Callbacks::on_new_frame(KeyFrame)           superlio / lightning)
        v
+------------------------------------------------+
| ExtensionModule（运行时 dlopen）                |
|   rviz_viewer        可视化 / TF                |
|   imu_prediction     IMU 插帧高频里程计          |
|   optimization_loam  SC-A-LOAM 式后端           |
|   optimization_miao  位姿图后端                 |
+------------------------------------------------+
```

## 数据流水线

1. 三种 ROS 入口节点持有一个 `AsukaROS` 实例；
2. `AsukaROS` 构造时读取 `config.json`，按 `config_odometry` 指定的动态库加载前端里程计插件，
   并将其封装为 `AsyncOdometryEstimation` 异步执行器；
3. IMU 消息在 ROS 层转换为 `asuka::ImuData::ConstPtr`（stamp + linear_acc + angular_vel），
   应用 `acc_scale` / `imu_time_offset` 后进入异步队列；
4. 点云消息按 `lidar_type` 转换为 `LivoxPoint` / `RobosensePoint`，经 `asuka::CloudPreprocess`
   （近/远距离滤除、抽稀）生成 `PointCloudT` 后入队；
5. 前端 worker 批量取出数据喂给里程计，里程计计算结果以 `KeyFrame` 通过
   `Callbacks::on_new_frame` 分享，扩展模块据此进行可视化、IMU 插帧与后端优化。

## 文档

**入门**：[安装](installation.md) / [Docker](docker.md) / [快速上手](quickstart.md)

**速查**：[参数速查](parameters.md) / [点云格式](point_type.md) / [核心 API](api.md) / [FAQ](faq.md)

**指南**：[里程计插件](odometry.md) / [扩展开发](extend.md) / [内置扩展](extensions.md) / [架构总览](architecture.md)

**深入阅读**（按模块组织的专题页，源自仓库 wiki）：

- 项目入口与运行形态 / ROS 封装与数据接入主链路 / 核心框架
- 里程计算法实现（五种算法逐文件精读） / 算法功能模块
- 插件机制与基础设施 / 配置边界 / 跨模块边界与数据流 / 第三方依赖
