# Parameters

asuka 的参数系统由一个总入口 `config/config.json` 路由到各组件的 JSON 文件，
**代码中一律通过 `asuka::Config` / `asuka::GlobalConfig` 读取，禁止手工 JSON 解析**；
各 odometry 插件的参数集中在 `asuka::xxx::OdometryEstimationParam` 结构体中，插件构造时
经参数系统一次性读入内部持有的 `param` 成员；其他模块的参数在构造时读入并直接作为成员变量。

## config.json —— 总路由

```json
{
  "global": {
    "config_ros":        "config_ros.json",
    "config_odometry":   "odometry/config_fastlivo.json",
    "config_extensions": "config_extensions.json"
  },
  "logging": { ... }
}
```

`global` 的各键值是相对 `config/` 目录的文件名。**切换前端算法只需修改
`config_odometry` 指向的文件**（`odometry/config_{fastlio,batchlio,smallpointlio,superlio,fastlivo,srlivo,zlio}.json`）。

总路由 `config.json` 所在目录可用 ROS 参数 `config_path` 覆盖（约定该目录下存在
`config.json`）；未设置或为空时回退到 `asuka` 包内 `config/` 目录。

## config_ros.json —— ROS 层

| 键 | 说明 |
|---|---|
| `lidar_type` | `livox`（PointCloud2 + livox 字段）、`livox_custom`（livox CustomMsg）或 `robosense`；决定点云反序列化路径 |
| `imu_topic` / `lidar_topic` | 订阅话题 |
| `image_topic` | 图像订阅话题（空 = 不订阅） |
| `acc_scale` | IMU 加速度计刻度（ROS 层缩放后进前端） |
| `imu_time_offset` | 加到 IMU 时间戳上的偏移（秒） |
| `time_offset_lidar_to_imu` | 加到点云时间戳上的偏移（秒） |
| `enable_map_saving` | 是否保存地图（对全部前端统一门控） |
| `map_saving_path` | 退出时保存地图的路径 |

!!! warning "lidar_type × 数据实际格式"
    `lidar_type` 决定 ROS 层的反序列化路径，必须与数据实际格式一致——六前端均已实现
    `livox` / `livox_custom` / `robosense` 三种格式的预处理重载，无需按前端挑选。
    格式不符（如把 CustomMsg 话题配成 `robosense`）或点字段缺失时，帧会被预处理
    拒绝并打出节流告警（`lidar frames dropped ...`）。

## odometry/config_*.json —— 前端算法（含各自的传感器外参）

每种算法一个文件，**雷达/相机外参也随算法走**（历史上的共享 `config_sensor.json`
已被此模式取代），以 `config_fastlio.json` 的 `sensor` 块为例：

```json
"sensor": {
  "extrinsic_T": [-0.011, -0.02329, 0.04412],
  "extrinsic_R": [1, 0, 0, 0, 1, 0, 0, 0, 1]
}
```

缺失时回退到各算法内置默认值（即其上游基准外参，如 smallpointlio 为 MID360 值）。
fastlivo 额外有 `camera` 块（`image_size` / `scale` / 内参 / 畸变 / `T_imu_camera`
TUM 七元），仅 LIVO 模式消费。

以 `config_fastlio.json` 为例的整体结构：

```json
{
  "odometry":   { "so_name": "libasuka_odometry_fastlio.so",
                  "imu_buffer_capacity": 1000, "lidar_buffer_capacity": 50,
                  "max_iteration": 3, "laser_point_cov": 0.001 },
  "preprocess": { "min_distance": 0.5, "max_distance": 100.0,
                  "point_filter_num": 3, "scan_line": 4, "scan_rate": 10 },
  "mapping":    { "gravity_estimation": true, "fov_degree": 360 },
  "filter":     { "filter_size_surf": 0.15, "filter_size_map": 0.25,
                  "cube_side_length": 1000 },
  "sensor":     { "extrinsic_T": [...], "extrinsic_R": [...] }
}
```

- `so_name`：要 `dlopen` 的里程计插件名；
- 其余键按模块分组，各算法文件内容不同（如 batchlio 有 `batch` / `mapping.ivox_*` 组，
  superlio 有 `kf` / `hash_map` 组），以文件内注释为准——多数键带原版基准标注；
- 地图保存由 `config_ros.json` 的 `ros.enable_map_saving` 统一门控（对全部前端一致）。
  各前端均为全量累积语义：fastlio / fastlivo 按上游 `pcl_wait_save` 语义逐帧追加
  全量去畸变扫描，batchlio / superlio 逐帧累加世界系点云，smallpointlio 在 ivox 插入点
  同步累加；
- 缺失的键回退到各算法内置默认值并记录 warning；关键参数建议用 `param_cast` 显式失败。

## config_extensions.json —— 扩展

`extensions.loading_extension_modules` 列出要加载的扩展动态库（顺序即加载顺序），
其余顶层键是各扩展自己的参数（`rviz_viewer` / `imu_prediction` /
`optimization_loam`），详见 [内置扩展](extensions.md)。

## 日志

`config.json` 的 `logging` 段：`console_output` / `console_level` / `file_output` /
`logging_dir` / `logging_level` / `rotate_logs` / `max_file_size_kb` / `max_files`。
每个模块（`ros` / `odometry` / `preprocess` / `rviz` / `imu` / `loam` …）经
`create_module_logger()` 获得独立 logger，同时写控制台与滚动文件。

## 代码中的读取方式

```cpp
const Config config = load_config("config_extensions");
world_frame_id = config.param<std::string>("rviz_viewer", "world_frame_id", "world");
so_name        = odometry_config.param_cast<std::string>("odometry", "so_name");
```

`Config` 支持 `param`（带默认值）/ `param_cast`（缺失即 `abort`）/
`param_nested`（嵌套键）/ `override_param`；`ConfigTraits` 内置
`Eigen::Vector3d` / `Matrix3d` / `Quaterniond` / `Isometry3d` 及 vector 类型的转换。
