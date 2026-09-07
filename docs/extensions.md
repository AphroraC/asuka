# Bundled extensions

所有内置扩展的参数位于 `config/config_extensions.json`，与前端算法解耦——
**任何扩展对五种前端算法均可用**。

## rviz_viewer（`libasuka_rviz_viewer.so`）

把关键帧转换为 ROS 消息：里程计、点云与 TF。同步工作在回调线程完成。

| 参数 | 默认 | 说明 |
|---|---|---|
| `world_frame_id` / `imu_frame_id` | `world` / `imu`（当前配置 `camera_init` / `body`） | TF 与消息坐标系 |
| `publish_odometry` | true | `/asuka/odometry` |
| `publish_odometry_imu` | true | `/asuka/odometry_imu`（高频预测流） |
| `publish_odometry_opt` | true | `/asuka/odometry_opt` |
| `publish_cloud_imu` / `publish_cloud_world` | true | `/asuka/points` / `/asuka/world` |
| `publish_tf` | true | TF 归属：若高频流开启则让位给 `imu_prediction`，否则逐关键帧发布 |

## imu_prediction（`libasuka_imu_prediction.so`）

复刻 Super-LIO 前向积分路径：以最新关键帧为锚点（位姿、速度、偏置），对 IMU 样本做
前向传播，产出 IMU 频率的高频里程计（`on_odometry_imu`），并拥有高频 TF。

| 参数 | 默认 | 说明 |
|---|---|---|
| `gravity` | 9.81 | 重力大小；重力方向固定为 (0,0,-g)（原版为在线估计，见源码注释） |
| `imu_scale` | 1.0 | 加速度刻度（原版为初始化阶段在线估计） |

实现细节：独立的处理线程 + 4096 深队列；锚点超前到达的样本（如 rosbag 全速回放）经
`pending_imus` 缓冲按时间序补积分。

## optimization_loam（`libasuka_optimization_loam.so`，需 `BUILD_OPTIMIZATION_MODULES=ON`）

SC-A-LOAM 风格后端：ScanContext 回环检测（距离校验 + ICP 验证）+ GTSAM iSAM2 增量
位姿图优化，结果经 `on_odometry_opt` 发布。回调只入队，检测/ICP/优化/可视化各占独立线程。

主要参数：`keyframe_meter_gap` / `keyframe_deg_gap`（关键帧间距）、`sc_dist_thres` /
`sc_max_radius`（回环检测）、`history_keyframe_search_radius` / `_time_diff` / `_num`、
`loop_noise_score`、`graph_update_times`、`loop_fitness_score_threshold`、
`loop_closure_frequency` / `graph_update_frequency` / `vizmap_frequency`、`publish_rate`。

## optimization_miao（`libasuka_optimization_miao.so`，需 `BUILD_OPTIMIZATION_MODULES=ON`）

基于 lightning 图优化库的后端：NDT 配准回环（`loop_kf_gap` / `ndt_score_th`）+
鲁棒核位姿图（Cauchy），支持高度先验（`with_height` / `height_noise`）。

主要参数：`loop_kf_gap` / `min_id_interval` / `closest_id_th` / `max_range` /
`ndt_score_th`、`motion_trans_noise` / `motion_rot_noise` / `loop_trans_noise` /
`loop_rot_noise`、`rk_loop_th`、`with_height` / `height_noise`、`publish_rate` / `verbose`。
