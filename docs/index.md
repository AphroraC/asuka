# Home

**asuka** 是一个稳健、精简、高效、可扩展的 LiDAR-Inertial(-Visual) SLAM 框架（ROS1 / catkin）。
它将六种 LiDAR-Inertial(-Visual) 里程计算法统一到同一套插件接口之下，通过全局回调插槽把前端状态
分享给运行时加载的扩展模块（后端位姿图优化、高频 IMU 预测、可视化等）。

仓库为双包结构：**`asuka`**（ROS-free 算法库：核心框架、六前端插件、算法扩展、配置）与
**`asuka_ros`**（ROS 适配层：传感器接入、rviz_viewer、三个入口节点）。

## 特性

- ***算法即插件*** —— FAST-LIO2、Batch-LIO、Small Point-LIO、Super-LIO、FAST-LIVO2、sr_livo
  六种前端里程计各自编译为独立动态库，运行时经 `dlopen` 加载，修改 `config.json` 即可切换算法。
- ***视觉输入就绪*** —— `insert_image` 视觉输入接口与
  `on_insert_image` 回调槽；fastlivo 前端在该模式下支持 LiDAR-Visual-Inertial 紧耦合（LIVO），
  点类型为带 rgb 的 `asuka::PointXYZITRGB`。
- ***稳健*** —— 统一的 `KeyFrame` 输出契约；回调插槽逐观察者隔离异常；每个算法独立的参数文件
  （含各自的雷达/相机外参 `sensor` 块）；框架层统一处理时间回绕检测、地图保存与优雅退出。
- ***精简高效*** —— 32 字节紧凑点类型 `asuka::PointXYZITRGB`（xyz + intensity + rgb +
  curvature 时间），替代 48 B 的 `pcl::PointXYZINormal`；点内仅保留 xyz、intensity
  与毫秒级时间偏移。
- ***可扩展*** —— 核心回调插槽（`asuka::Callbacks` 六个）暴露 IMU / 帧 / 图像输入、
  关键帧与两条扩展输出流；freedom 扩展演示了扩展自持私有插槽（`FreedomCallbacks`）的
  模式。扩展模块即共享库，无需修改框架代码。

## 总体架构

```
asuka_rosnode / asuka_rosbag / asuka_nodelet      <- ROS 入口（asuka_ros 包，三选一）
        |  sensor_msgs::Imu / PointCloud2 / livox CustomMsg / Image
        v
    AsukaROS                                       <- ROS 层（asuka_ros 包）：消息转换、
        |  asuka::ImuData / PointCloudT / ImageData    acc_scale、时间偏移、点云预处理、
        v                                              扩展加载
AsyncOdometryEstimation                            <- 异步执行器（前端 worker 线程，
        |                                              imu/frame/image 三路有界队列）
        v
asuka::OdometryEstimation   <== dlopen ==   libasuka_odometry_*.so
        |                                             (fastlio / batchlio / smallpointlio /
        |  Callbacks::on_new_frame(KeyFrame)           superlio / fastlivo /
        |                                              srlivo)
        v
+------------------------------------------------+
| ExtensionModule（运行时 dlopen）                |
|   rviz_viewer        可视化 / TF（asuka_ros 包） |
|   imu_prediction     IMU 插帧高频里程计          |
|   freedom            动态物去除管线（on_freedom_*）|
|   optimization_loam  SC-A-LOAM 式后端           |
+------------------------------------------------+
```

## 数据流水线

1. 三种 ROS 入口节点（`asuka_ros` 包）持有一个 `AsukaROS` 实例；
2. `AsukaROS` 构造时读取 `config.json`，经 `OdometryEstimation::load_configured()` 按
   `config_odometry` 指定的动态库加载前端里程计插件，封装为 `AsyncOdometryEstimation`，
   再经 `ExtensionModule::load_configured_modules()` 加载扩展；
3. IMU 消息在 ROS 层转换为 `asuka::ImuData::ConstPtr`，应用 `acc_scale` / `imu_time_offset`
   后进入异步队列；
4. 点云按 `lidar_type`（`livox` / `livox_custom` / `robosense`）反序列化为 `LivoxPoint` /
   `LivoxCustomPoint` / `RobosensePoint`，经各前端专属的 `asuka::CloudPreprocess` 生成
   `PointCloudT` 后入队；图像消息转换为 `asuka::ImageData`
   进入第三条队列；
5. 前端 worker 按传感器时间归并逐条取出数据，先发射 `on_insert_imu` / `on_insert_frame` /
   `on_insert_image` 再喂给里程计；里程计结果以 `KeyFrame` 通过 `Callbacks::on_new_frame`
   分享，扩展模块据此进行可视化、IMU 插帧与后端优化。

## 文档

**入门**：[安装](installation.md) / [Docker](docker.md) / [快速上手](quickstart.md)

**速查**：[参数速查](parameters.md) / [点云格式](point_type.md) / [核心 API](api.md) / [FAQ](faq.md)

**指南**：[里程计插件](odometry.md) / [扩展开发](extend.md) / [内置扩展](extensions.md) / [架构总览](architecture.md)

**深入阅读**（按模块组织的专题页，源自仓库 wiki；注意其为历史快照，见各页顶部声明）：

- 项目入口与运行形态 / ROS 封装与数据接入主链路 / 核心框架
- 里程计算法实现（逐文件精读） / 算法功能模块
- 插件机制与基础设施 / 配置边界 / 跨模块边界与数据流 / 第三方依赖
