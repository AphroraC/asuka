# 里程计性能基准（2026-09-30，2026-10 增补横评）

现役前端的同机串行回放测量。方法：`asuka_rosbag` 全速回放（含背压门），`config_path`
覆盖法逐前端切换路由，`enable_map_saving: true`，扩展仅 `rviz_viewer`（默认配置，发布
开销计入），`acc_scale: 9.80665`。CPU 计时为 bash `time` 的 user+sys。

## 测试素材

| bag | 时长 | 内容 |
|---|---|---|
| `mapping.bag` | 529.2 s | livox PointCloud2（`lidar_type: livox`），手持扫描 |
| `hku.bag` | 91.0 s | livox CustomMsg + `/left_camera/image` 10 Hz（LIVO 素材） |

## 结果总表

| 前端 | 模式 | bag | 墙钟 | 整体实时率* | CPU (user+sys) | CPU 利用率 | 存图点数 | worker 异常 |
|---|---|---|---|---|---|---|---|---|
| fastlio | LIO | mapping | 178 s | 2.97x | 159.6 s | 0.90 核 | 28,750,914 | 0 |
| superlio | LIO | mapping | 63 s | **8.44x** | 117.9 s | **1.88 核** | 9,229,819 | 0 |
| fastlivo | ONLY_LIO | mapping | 400 s | 1.32x | 352.0 s | 0.88 核 | 79,524,778 | 0 |
| fastlivo | LIVO | hku | 62 s | 1.47x | 55.3 s | 0.89 核 | 11,402,819 | 0 |

\* 整体实时率 = bag 时长 / 墙钟（含 rosrun 启动开销）；单次运行，异步调度抖动带约 ±0.3%。
共享容器背景下测量，数值供横向量级参考。

## 解读

- **吞吐**：superlio 最快（8.4x，测于其 tbb 并行时代，CPU 利用率 1.88 核——tbb 现已整体
  移除，见下文 2026-10-26 注记）；fastlio / fastlivo ONLY_LIO 在 1.3-3.0x 一档，均为近
  单核利用（0.88-0.90 核）。
- **存图点数反映各自 `save_map` 语义，不可作精度横向比较**：fastlivo ONLY_LIO 为上游
  `pcl_wait_save` 全量逐帧累积（79.5M），fastlio 同为全量累积（28.8M），superlio 逐帧
  累加世界系点云。语义细节见[里程计插件](odometry.md)。
- **稳定性**：四组回放全部 EXIT=0、worker 异常 0；fastlio 的存图点数与既有回归基线一致
  （28.75M 复现）。
- **LIVO 前端 × 无图 bag 的误配症状**：slam_mode=LIVO 而 bag 无图像时，同步器永久等图、
  内部雷达缓冲积压触发回放背压，表现为 ~0.2x 慢速空转（CPU 近零）——见 [FAQ](faq.md)。

## 复现

配置准备用 `config_path` 覆盖法：拷贝 `config/` 到临时目录，改 `config.json` 的
`config_odometry` 路由、`config_ros.json` 的 `lidar_type` / `image_topic` /
`enable_map_saving` / `map_saving_path`（LIVO 前端 × 无图 bag 还需把 `slam_mode` 改
`ONLY_LIO`），`rosparam set /asuka_rosbag/config_path <dir>` 后全速回放。墙钟与 CPU
用 bash `time` 记录，吞吐从日志 `playback speed` 行与 bag 时长换算。

## 2026-10 增补：精度-资源横评

方法同上（`config_path` 覆盖、全速回放），新增三项计量：CPU/峰值 RSS 由
`getrusage(RUSAGE_CHILDREN)` 一次取得；轨迹经 `rosbag record /asuka/odometry` 记录后，
以 fastlio（LIVO 组为 fastlivo）轨迹为参考按时间戳插值计算位置 RMSE；
`enable_map_saving: false`——峰值 RSS 反映运行态内存（不含存图），CPU 与上表不可直接比。

liantiao.bag（116.5 s，2026-10-23，fastlio 为参考）：

| 前端 | 墙钟 | 实时率 | CPU (s) | 平均 CPU | 峰值 RSS | RMSE vs fastlio | 终点漂移 |
|---|---|---|---|---|---|---|---|
| fastlio | 35.7 s | 3.26x | 31.0 | 86.8% | 162 MiB | —（参考） | 0.567 m |
| superlio | 13.0 s | 8.96x | 20.7 | 159.2% | 65 MiB | 2.77 cm | 0.561 m |

两前端终点闭环漂移 0.561-0.567 m 相互吻合（fastlio 0.567 复现 2026-09-26 的 0.566 记录）。

hku.bag（91.0 s，2026-10-25，LIVO 前端，fastlivo 为参考；参考轨迹须按时间戳去重——
同一扫描的 LIO/VIO 双帧共享 stamp）：

| 前端 | 墙钟 | 实时率 | CPU (s) | 平均 CPU | 峰值 RSS | RMSE | 终点漂移 |
|---|---|---|---|---|---|---|---|
| fastlivo | 64.5 s | 1.41x | 57.1 | 88.5% | 726 MiB | —（参考） | 0.01 m |

## 2026-10-26 增补：liantiao 横评（现役三前端）

方法与 09-30 总表一致：`config_path` 覆盖法、`enable_map_saving: false`（运行时内存语义）、
全速回放、扩展仅 `rviz_viewer`（默认配置）。superlio 行测于 tbb 并行时代（此后同日落地上限、
当晚 tbb 整体移除——三处 parallel_for 换为 `ENABLE_OMP`/`NUM_THREADS` 编译开关控制的
OpenMP（CMake 默认关闭=纯串行，轨迹与串行逐位一致），asuka 不再依赖 tbb）。
**同夜去畸变越界分支修复（对齐上游钳位语义）后复测：RMSE 0.81 cm / 中位 0.65 / 最大 2.88 cm、
终点 0.559 m**——下表 superlio 行（2.77 cm）测于该修复前；修复同时消除了 rviz 中扫描帧偶发
大角度倾斜与建图结构倾斜的缺陷（越界点曾被按整机世界姿态角旋转，liantiao 1164/1165 帧有
0.5%-12.4% 的点走该分支）。CPU 与峰值 RSS
用 `getrusage(RUSAGE_CHILDREN)`。RMSE 对 fastlio 参考轨迹按时间戳插值；终点漂移为里程计
终点距原点距离（liantiao 为闭环 bag）。

| 前端 | 墙钟 | 整体实时率 | CPU (user+sys) | CPU 利用率 | 峰值 RSS | RMSE | 中位 | 最大 | 终点漂移 |
|---|---|---|---|---|---|---|---|---|---|
| fastlio（参考） | 35.8 s | 3.25x | 42.1 s | 1.18 核 | 217 MiB | — | — | — | 0.568 m |
| superlio | 14.7 s | 7.93x | 44.8 s | 3.05 核 | 127 MiB | 2.77 cm | 2.32 cm | 6.6 cm | 0.561 m |
| fastlivo（ONLY_LIO） | 67.8 s | 1.72x | 72.6 s | 1.07 核 | 659 MiB | 323.67 cm | 227.62 cm | 668.2 cm | 0.561 m |

- superlio 的 RMSE 与 2026-10-23 横评逐位一致（2.77 cm），参考终点漂移 0.568 m 同样复现
  0.567 m；三组回放全部 EXIT=0。
- fastlivo ONLY_LIO 在该 bag 上中途漂移显著（最大 6.7 m），终点随 bag 闭环收敛回 0.561 m，
  为该 bag 弱档；峰值 RSS 亦最高（ikd-tree 与全量特征缓冲，量级与 09-30 mapping.bag
  ONLY_LIO 的 726 MiB 相符）。
