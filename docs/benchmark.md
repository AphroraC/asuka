# 里程计性能基准（2026-09-30）

全部六个前端的同机串行回放测量。方法：`asuka_rosbag` 全速回放（含背压门），`config_path`
覆盖法逐前端切换路由，`enable_map_saving: true`，扩展仅 `rviz_viewer`（默认配置，发布
开销计入），`acc_scale: 9.80665`。CPU 计时为 bash `time` 的 user+sys。

## 测试素材

| bag | 时长 | 内容 |
|---|---|---|
| `mapping.bag` | 529.2 s | livox PointCloud2（`lidar_type: livox`），手持扫描 |
| `hku.bag` | 91.0 s | livox CustomMsg + `/left_camera/image` 10 Hz（LIVO 素材） |

## 结果总表

> 时效注记（2026-10-23）：smallpointlio 已从 asuka 移除；下表及其解读中的相应行保留为
> 2026-09-30 的历史测量。

| 前端 | 模式 | bag | 墙钟 | 整体实时率* | CPU (user+sys) | CPU 利用率 | 存图点数 | worker 异常 |
|---|---|---|---|---|---|---|---|---|
| fastlio | LIO | mapping | 178 s | 2.97x | 159.6 s | 0.90 核 | 28,750,914 | 0 |
| batchlio | LIO (output) | mapping | 113 s | 4.70x | 90.1 s | 0.80 核 | 10,995,673 | 0 |
| smallpointlio | LIO | mapping | 171 s | 3.09x | 121.9 s | 0.71 核 | 37,833,693 | 0 |
| superlio | LIO | mapping | 63 s | **8.44x** | 117.9 s | **1.88 核** | 9,229,819 | 0 |
| fastlivo | ONLY_LIO | mapping | 400 s | 1.32x | 352.0 s | 0.88 核 | 79,524,778 | 0 |
| fastlivo | LIVO | hku | 62 s | 1.47x | 55.3 s | 0.89 核 | 11,402,819 | 0 |
| srlivo | LIVO | hku | 66 s | 1.39x | 55.0 s | 0.84 核 | 1,254,693 | 0 |

\* 整体实时率 = bag 时长 / 墙钟（含 rosrun 启动开销）；单次运行，异步调度抖动带约 ±0.3%。
共享容器背景下测量，数值供横向量级参考。

## 解读

- **吞吐**：superlio 最快（8.4x，且是唯一用满多核的前端，tbb 并行使 CPU 利用率 1.88 核）；
  batchlio 次之（4.7x）；fastlio / smallpointlio / fastlivo ONLY_LIO 在 1.3-3.1x 一档，
  均为近单核利用（0.7-0.9 核）。srlivo 受图像流驱动（测量窗图像锚定），1.4x。
- **存图点数反映各自 `save_map` 语义，不可作精度横向比较**：fastlivo ONLY_LIO 为上游
  `pcl_wait_save` 全量逐帧累积（79.5M），smallpointlio 为 ivox 插入点全量累加（37.8M），
  fastlio 同为全量累积（28.8M），batchlio / superlio 逐帧累加世界系点云，srlivo 为
  色图过滤输出（1.25M，`pub_point_minimum_views`）。语义细节见[里程计插件](odometry.md)。
- **稳定性**：七组回放全部 EXIT=0、worker 异常 0；fastlio / srlivo / fastlivo×hku 的
  存图点数与既有回归基线一致（fastlio 28.75M 复现、srlivo 1,254,693 = 历史精确基线）。
- **LIVO 前端 × 无图 bag 的误配症状**：slam_mode=LIVO 而bag 无图像时，同步器永久等图、
  内部雷达缓冲积压触发回放背压，表现为 ~0.2x 慢速空转（CPU 近零）——见 [FAQ](faq.md)。

## 复现

配置准备用 `config_path` 覆盖法：拷贝 `config/` 到临时目录，改 `config.json` 的
`config_odometry` 路由、`config_ros.json` 的 `lidar_type` / `image_topic` /
`enable_map_saving` / `map_saving_path`（LIVO 前端 × 无图 bag 还需把 `slam_mode` 改
`ONLY_LIO`），`rosparam set /asuka_rosbag/config_path <dir>` 后全速回放。墙钟与 CPU
用 bash `time` 记录，吞吐从日志 `playback speed` 行与 bag 时长换算。
