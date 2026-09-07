# Core API reference

头文件均在 `include/asuka/` 下，符号位于 `namespace asuka`。

## 数据类型（core/types.hpp）

| 类型 | 说明 |
|---|---|
| `ImuData` | `stamp` + `linear_acc` + `angular_vel`（`Ptr` / `ConstPtr`） |
| `PointXYZIOffset` / `PointT` / `PointCloudT` | 32 B 紧凑点类型，见[点云格式](point_type.md) |
| `LivoxPoint` / `RobosensePoint` | 原始雷达点（含 `double timestamp`） |
| `KeyFrame` | 帧级输出契约，见[扩展开发](extend.md) |
| `LidarType` / `parse_lidar_type()` | `LIVOX` / `ROBOSENSE` |

## Config / GlobalConfig（utility/config.hpp）

| API | 说明 |
|---|---|
| `GlobalConfig::instance(config_dir, override)` | 总配置单例（`config/config.json`） |
| `GlobalConfig::get_config_path(name)` | 由 `global` 段路由出组件配置文件路径 |
| `Config::param<T>(module, key, default)` | 读取参数，缺失回退默认值并 warning |
| `Config::param_cast<T>(module, key)` | 读取参数，缺失 `abort`（必选参数用这个） |
| `Config::param_nested<T>(modules, key)` | 嵌套键读取（如 sensor 块） |
| `Config::override_param / save / has_param` | 覆写、落盘、存在性查询 |

支持类型含数值、字符串、`std::vector`、`Eigen::Vector3d/4d`、`Matrix3d`、
`Quaterniond`、`Isometry3d`（见 `config_impl.hpp` 的 `ConfigTraits`）。

## CallbackSlot / Callbacks（utility/callback_slot.hpp, core/callbacks.hpp）

| API | 说明 |
|---|---|
| `CallbackSlot<Func>::add(cb) -> int` | 注册观察者，返回稳定 id |
| `CallbackSlot<Func>::remove(id)` | 注销（留空洞，id 不复用） |
| `CallbackSlot<Func>::call(args...)` | 发射（无锁快照遍历，逐观察者异常隔离） |
| `Callbacks::on_insert_imu / on_insert_frame / on_new_frame / on_odometry_imu / on_odometry_opt` | 五个全局插槽 |

## OdometryEstimation（core/odometry_estimation.hpp）

见[里程计插件](odometry.md)。前端插件须实现
`create_odometry_estimation()` C 工厂。

## CloudPreprocess / ImuPreprocess（core/*.hpp）

| 类 | 接口 | 说明 |
|---|---|---|
| `CloudPreprocess` | `preprocess(stamp, LivoxPoint/RobosensePoint cloud) -> PointCloudT::Ptr` | 距离滤除、抽稀；每算法派生，由算法创建、框架持有并调用 |
| `ImuPreprocess` | `insert_imu / initialize / is_initialized / get_mean_acceleration / get_mean_gyroscope` | IMU 静态初始化与去畸变的算法侧基类 |

## ExtensionModule（utility/extension_module.hpp）

| API | 说明 |
|---|---|
| `stop()` | join 自有线程、停止槽发射；**必须幂等**，默认空实现 |
| `at_exit(dump_path)` | 退出收尾（如轨迹/地图导出），在全部 stop 之后调用 |
| `ExtensionModule::load_module(so_name)` | `dlopen` 并调用 `create_extension_module()` |

## AsukaROS / AsyncOdometryEstimation（asuka_ros.hpp, core/async_odometry_estimation.hpp）

| API | 说明 |
|---|---|
| `AsukaROS(nh)` | 完成配置加载、插件加载、扩展加载 |
| `insert_imu / insert_frame` | ROS 消息入口 |
| `workload() / wait(auto_quit)` | 背压与排空等待 |
| `save(path)` | 停 worker 并保存地图 |
| `extensions()` | 已加载扩展列表 |
| `AsyncOdometryEstimation::stop() / wait_idle()` | 执行器停止 / 空闲等待 |
