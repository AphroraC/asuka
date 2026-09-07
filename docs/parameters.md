# Parameters

asuka 的参数系统由一个总入口 `config/config.json` 路由到各组件的 JSON 文件，
**代码中一律通过 `asuka::Config` / `asuka::GlobalConfig` 读取，禁止手工 JSON 解析**；
模块参数在构造时读入并直接作为成员变量。

## config.json —— 总路由

```json
{
  "global": {
    "config_ros":        "config_ros.json",
    "config_sensor":     "config_sensor.json",
    "config_odometry":   "odometry/config_fastlio.json",
    "config_extensions": "config_extensions.json"
  },
  "logging": { ... }
}
```

`global` 的各键值是相对 `config/` 目录的文件名。**切换前端算法只需修改
`config_odometry` 指向的文件**，例如 `odometry/config_batchlio.json`。

## config_ros.json —— ROS 层

| 键 | 说明 |
|---|---|
| `lidar_type` | `livox` 或 `robosense`，决定点云反序列化格式 |
| `imu_topic` / `lidar_topic` | 订阅话题 |
| `acc_scale` | IMU 加速度计刻度（ROS 层统一缩放） |
| `imu_time_offset` | 加到 IMU 时间戳上的偏移（秒） |
| `time_offset_lidar_to_imu` | 加到点云时间戳上的偏移（秒） |
| `enable_map_saving` / `map_saving_path` | 退出时保存地图 |

## config_sensor.json —— 传感器外参与噪声

按雷达名分块（`livox` / `robosense`），包含 `extrinsic_T` / `extrinsic_R`、
`acc_noise` / `gyro_noise` / `acc_bias` / `gyro_bias` 等。**未启用的块必须注释掉**：
各算法按 `has("livox") -> has("robosense")` 的优先级取第一个存在的块，
该选择与 `config_ros.json` 的 `lidar_type` 相互独立，两者务必保持一致（见 [FAQ](faq.md)）。

## odometry/config_*.json —— 前端算法

每种算法一个文件。以 `config_fastlio.json` 为例：

```json
{
  "odometry":   { "so_name": "libasuka_odometry_fastlio.so",
                  "imu_buffer_capacity": 1000, "lidar_buffer_capacity": 50,
                  "max_iteration": 3, "laser_point_cov": 0.001 },
  "preprocess": { "min_distance": 0.5, "max_distance": 100.0,
                  "point_filter_num": 3, "scan_line": 4, "scan_rate": 10 },
  "mapping":    { "gravity_estimation": true, "fov_degree": 360 },
  "filter":     { "filter_size_surf": 0.15, "filter_size_map": 0.25,
                  "cube_side_length": 1000 }
}
```

- `so_name`：要 `dlopen` 的里程计插件名；
- 其余键按模块分组（`odometry` / `preprocess` / `mapping` / `filter`），
  各算法文件内容不同（如 batchlio 有 `mapping.ivox_grid_resolution` 等），以文件内为准。
- 缺失的键回退到各算法内置默认值并记录 warning；关键参数建议用 `param_cast` 显式失败。

## config_extensions.json —— 扩展

`extensions.loading_extension_modules` 列出要加载的扩展动态库（顺序即加载顺序），
其余顶层键是各扩展自己的参数（`rviz_viewer` / `imu_prediction` / `optimization_loam` /
`optimization_miao`），详见 [内置扩展](extensions.md)。

## 日志

`config.json` 的 `logging` 段：`console_output` / `console_level` / `file_output` /
`logging_dir` / `logging_level` / `rotate_logs` / `max_file_size_kb` / `max_files`。
每个模块（`ros` / `odometry` / `preprocess` / `rviz` / `imu` / `loam` …）经
`create_module_logger()` 获得独立 logger，同时写控制台与滚动文件。

## 代码中的读取方式

```cpp
const Config config(GlobalConfig::get_config_path("config_extensions"));
world_frame_id = config.param<std::string>("rviz_viewer", "world_frame_id", "world");
so_name        = odometry_config.param_cast<std::string>("odometry", "so_name");
```

`Config` 支持 `param`（带默认值）/ `param_cast`（缺失即 `abort`）/
`param_nested`（嵌套键）/ `override_param`；`ConfigTraits` 内置
`Eigen::Vector3d` / `Matrix3d` / `Quaterniond` / `Isometry3d` 及 vector 类型的转换。
