# Bundled extensions

所有内置扩展的参数位于 `config/config_extensions.json`，与前端算法解耦——
**任何扩展对六种前端算法均可用**。注意 `rviz_viewer` 的动态库随 `asuka_ros` 包发布
（它是本仓唯一依赖 ROS 的扩展）。

## rviz_viewer（`libasuka_rviz_viewer.so`）

把关键帧转换为 ROS 消息：里程计、点云与 TF。回调只入队，转换与发布在模块自己的处理线程上完成。

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

输入 IMU 流已经过 ROS 层 `acc_scale` 单位归一（见 [参数](parameters.md)），本模块不做
二次缩放。

实现细节：独立的处理线程 + 4096 深队列；锚点超前到达的样本（如 rosbag 全速回放）经
`pending_imus` 缓冲按时间序补积分。
## freedom（`libasuka_freedom.so`）

FreeDOM（RA-L 2025）动态物去除管线的 ROS-free 移植。**前端无关**：消费 `on_new_frame`
的（去畸变 IMU 系点云 + 前端位姿）二元组——该输入契约对任何前端成立，因此对六种前端
通用。三个阶段（扫描去除 → raycast 增强 → 地图去除）在模块自己的处理线程上依次运行，
并经 `on_freedom_scan_removal` / `on_freedom_raycast_enhancement` /
`on_freedom_map_removal` 三个插槽发布阶段结果——这三个插槽由本扩展**自持**
（`asuka::FreedomCallbacks`，静态对象定义于 `libasuka_freedom.so`，见
[扩展开发](extend.md)），不占用 core 的 `asuka::Callbacks` 总线。模块本身保持
ROS-free：**插槽是唯一 的对外接口**，ROS 侧消费方为 freedom_viewer 扩展（见下节）。`publish` 开关块逐流
控制结果生成——关闭的流在 payload 中为 null 字段。静态地图在 `at_exit()` 落盘
（`enable_map_saving` / `map_saving_path`）。

| 参数块 | 内容 |
|---|---|
| `sensor` | `min_range` / `max_range` / z 带过滤 |
| `map` | `sub_voxel_size`、`voxel_depth` / `block_depth`、局部地图开关与范围、raycast 范围与 z 带、`counts_to_free` / `counts_to_revert`、保守/激进连通性 |
| `raycast_enhancement` | 深度图线数与 FOV、增强范围、深度 margin、inpaint / erosion、FOV mask |
| `publish` | 各阶段话题逐个开关 |
| 其他 | `num_threads`、`enable_map_saving` / `map_saving_path` |

## freedom_viewer（`libasuka_freedom_viewer.so`，asuka_ros 包）

freedom 结果流的 ROS 发布器：订阅三个 `on_freedom_*` 插槽，把 payload 转成
`/asuka/freedom/*` 话题（14 个点云 + 2 张深度图 + 3 个 range box marker）。回调仅
入队，序列化与发布在自有处理线程上完成；payload 中为 null 的字段（对应 freedom
`publish` 块关闭的流）直接跳过。**使用方式**：与 `libasuka_freedom.so` 一起加入
`loading_extension_modules`。参数块 `freedom_viewer`：`world_frame_id`（默认
camera_init）、`topic_prefix`（默认 /asuka/freedom）。


## optimization_loam（`libasuka_optimization_loam.so`，需 `BUILD_OPTIMIZATION_MODULES=ON`）

SC-A-LOAM 风格后端：ScanContext 回环检测（距离校验 + ICP 验证）+ GTSAM iSAM2 增量
位姿图优化，结果经 `on_odometry_opt` 发布。回调只入队，检测/ICP/优化/可视化各占独立线程。

主要参数：`keyframe_meter_gap` / `keyframe_deg_gap`（关键帧间距）、`sc_dist_thres` /
`sc_max_radius`（回环检测）、`history_keyframe_search_radius` / `_time_diff` / `_num`、
`loop_noise_score`、`graph_update_times`、`loop_fitness_score_threshold`、
`loop_closure_frequency` / `graph_update_frequency` / `vizmap_frequency`、`publish_rate`。
