# Bundled extensions

所有内置扩展的参数位于 `config/config_extensions.json`，与前端算法解耦——
**任何扩展对三种前端算法均可用**。`rviz_viewer` 与 `imu_prediction` 的动态库均随
`asuka_ros` 包发布。

## rviz_viewer（`libasuka_rviz_viewer.so`）

把关键帧转换为 ROS 消息：里程计、点云与 TF。回调只入队，转换与发布在模块自己的处理线程上完成。

| 参数 | 默认 | 说明 |
|---|---|---|
| `world_frame_id` / `imu_frame_id` | `world` / `imu`（当前配置 `camera_init` / `body`） | TF 与消息坐标系 |
| `publish_odometry` | true | `/asuka/odometry` |
| `publish_odometry_imu` | true | `/asuka/odometry_imu`（高频预测流） |
| `publish_cloud_imu` / `publish_cloud_world` | true | `/asuka/points` / `/asuka/world` |
| `publish_tf` | true | TF 归属：若高频流开启则让位给 `imu_prediction`，否则逐关键帧发布 |

**切片语义**：`/asuka/points`、`/asuka/world` 是逐关键帧切片（当前扫描的 IMU 系/
世界系副本）；现役三前端均发射完整去畸变扫描，rviz 默认替换语义（Decay Time=0）下
显示连续。点云发布仅保留 `/asuka/points` 与 `/asuka/world`。

## imu_prediction（`libasuka_imu_prediction.so`）

复刻 Super-LIO 前向积分路径：以最新关键帧为锚点（位姿、速度、偏置），对 IMU 样本做
前向传播，产出 IMU 频率的高频里程计（`on_odometry_imu`），并拥有高频 TF。

| 参数 | 默认 | 说明 |
|---|---|---|
| `gravity` | 9.81 | 重力大小；重力方向固定为 (0,0,-g)（原版为在线估计，见源码注释） |

输入 IMU 流已经过 ROS 层 `acc_scale` 单位归一（见 [参数](parameters.md)），本模块不做
二次缩放。

实现细节：独立的处理线程 + 4096 深队列；锚点超前到达的样本（如 rosbag 全速回放）经
`pending_imus` 缓冲按时间序补积分。

（曾内置的 SC-A-LOAM 式后端 `optimization_loam` 已于 2026-10-26 整体移除，asuka 不再依赖
GTSAM；`on_odometry_opt` 槽与 `/asuka/odometry_opt` 话题亦随之移除（2026-10-27）——后端扩展
如需输出自己的轨迹流，应按[扩展开发](extend.md)自持私有插槽。）
