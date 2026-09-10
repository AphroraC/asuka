# smallpointlio 与原版等价性验证

本文记录 asuka `smallpointlio` 前端与上游 Small Point-LIO（`test/small_point_lio`，`ros2` 分支）
的等价性核对过程、结论与为此做的修改。验证工具在 `test/small_point_lio_offline/`
（原版源码零改动，通过 compat 头把 ROS2 依赖桩化后编成离线可执行文件），用法见该目录的 README。

## 一、验证方法

1. 用离线 harness 跑上游**未修改**的算法源码，输入同一份 rosbag（`/share/rosbag/liantiao.bag`，
   `/livox/lidar` + `/livox/imu`，116 s），逐条按到达顺序喂入并在每次回调后调用 `handle_once()`，
   复刻上游 ROS2 节点的单线程执行器；参数直接取自上游 `config/mid360.yaml`。
2. asuka 侧用 `asuka_rosbag` 回放同一份 bag（配置对齐后），导出 `/asuka/odometry`、
   `/asuka/points`（IMU 系关键帧点云）、`/asuka/world`。
3. `tools/compare_ab.py` 逐帧对比里程计（位置/姿态/速度）、关键帧点云点数与逐点坐标。

## 二、核对发现的差异（已修复）

### 2.1 执行器成批喂数据，算法行为随之改变（严重）

`AsyncOdometryEstimation` 原先把队列一次性取空、**先喂全部 IMU 再喂全部帧**，再排空 `process_once()`。
上游是"每条消息一次 `insert` + 一次 `handle_once`"。差别不是性能而是**语义**：初始化条件
`imu_deque.size() >= 200` 触发时，上游的 `imu_deque` 恰好 200 个样本，而成批喂入时会有更多；
初始重力方向正是对这批样本求均值，于是初值不同、整条轨迹随之偏移（实测末帧位置差 1.4 mm、
速度差 2.2 cm/s，且关键帧边界整体错开一帧）。

**修复**：`AsyncOdometryEstimation` 改为单条 FIFO（保留到达顺序），逐条喂入并在每条之后排空，
与上游逐回调语义一致。

### 2.2 `process_once()` 无进展也返回 true → worker 空转死锁（严重）

初始化门槛要求 `imu_deque >= 200`，而 `process_once()` 只要 `point_deque` 与 `imu_deque` 非空就返回
true，`handle_once` 在门槛未满足时不消费任何数据。`AsyncOdometryEstimation` 的
`while (process_once()) {}` 因此进入死循环，worker 再也不回到 `cv.wait` 取新数据 —— 门槛永远无法满足。
实测：本 bag 首帧前只有 19 个 IMU 样本，asuka 卡在 100% CPU 空转、**从不输出任何结果**，
而 `asuka_rosbag` 的背压（500 ms 无进展即跳出）把回放速度压到 0.197x。

**修复**：`handle_once()` 返回"本次是否消费了数据（或完成初始化）"，`process_once()` 返回该值；
无进展即返回 false，worker 回到等待状态。这同时消除了初始化期间的忙等。

### 2.3 dense / filtered 双点集被塌缩成一个（严重）

上游 `Preprocess::on_point_cloud_callback` 维护两个集合：

* `dense_points`：**全部点**（仅按 `timestamp >= last_timestamp_dense_point` 去重）→ 关键帧点云；
* `filtered_points`：`i % point_filter_num`、`timestamp >= last_timestamp_lidar`、min/max 距离带过滤，
  可选体素降采样 → 驱动 ESKF。

asuka 原先把**同一份已过滤+已降采样**的点云塞进两个 deque，于是关键帧点云变成稀疏降采样点，
且缺少两处 `last_timestamp_*` 去重。

**修复**：`smallpointlio::CloudPreprocess` 退化为纯格式转换（tag 过滤 + 时间偏移，与上游 adapter 一致，
保留消息顺序），上游 `Preprocess` 的全部逻辑下沉到 `OdometryEstimation::insert_frame`
（worker 线程执行，只把入队部分放进 `buffer_mutex`）。过滤参数（min/max_distance、
point_filter_num、space_downsample、leaf）随之移入 `OdometryEstimation` 成员，与上游同为 float 语义。

> 注：这与最初"预处理全部放在 asuka_ros/ROS 线程"的设计不同 —— 关键帧点云所需的 dense 全量点集
> 只有算法侧能拿到，而 `asuka::CloudPreprocess` 的接口只返回一份点云。框架接口未改动。

### 2.4 `i % point_filter_num` 的下标基准

上游的下标是**tag 过滤后**的紧凑下标（tag 过滤在 adapter 里），asuka 原先用原始点云下标。
第 2.3 节的改动顺带修正：CloudPreprocess 先做 tag 过滤，算法再在紧凑序列上做抽稀。

### 2.5 ESKF 饱和门控用错标志

上游 `eskf.h` 用 `satu_check[i + 3]` 门控加速度行，asuka 用了 `satu_check[i]`（陀螺标志），
注释还声称"与原版一致"。已改为 `satu_check[i + 3]`。

### 2.6 `imu.satu_acc` 单位语义不一致（严重）

上游比较的是**原始 g 值**（阈值 `3.0*0.99 = 2.97` g）；asuka 在 ROS 层已把加速度
`× config_ros.acc_scale = 9.80665` 变成 m/s²，却仍拿 2.97 去比较。实测本 bag：

| 配置 | 判定为饱和的样本 |
|---|---|
| 修复前（2.97 比 m/s² 值） | 23289/23301 = **99.9%**（z 轴单独 63.7%） |
| 修复后（29.41995 比 m/s² 值） | 0（与上游一致：本 bag 的 |acc| 最大 1.4 g，未达 3 g） |

即修复前加速度残差与 `ba` 的协方差行几乎永久失效。修复方式：配置改用
`3.0 * 9.80665 = 29.41995`（代码仍按上游乘 0.99），并在配置文件里写明该耦合。

> 两层缩放（ROS 层 ×9.80665 × 算法层 ×(9.81/9.80665)）净效果等于上游的 ×9.81，
> 属于架构待办：更干净的做法是平台缩放只在算法内做，需要框架提供按算法 opt-out 的契约。

### 2.7 `publish_odometry_without_downsample` 分支点云泄漏

该开关为 true 时，`pointcloud_imu_frame` 永不清空（无界增长）且关键帧永远没有点云。
上游无论该开关如何都会在帧末产出并清空点云。已改为：帧末始终消费/清空缓存，
该开关为 true 时用一个 `id = -1`、仅携带点云与当前位姿的帧发出（逐点里程计帧不带点云）。

### 2.8 容器与死参数

点云/IMU 缓冲由 `boost::circular_buffer(50000/1000)` 改为 `std::deque`（上游语义，满时不会静默丢最旧）；
`odometry.imu_buffer_capacity` / `lidar_buffer_capacity` 因此不再被 smallpointlio 读取（保留键并在配置中注明）。

### 2.9 参数对齐（配置差异）

| 参数 | 上游 mid360.yaml | 修复前 asuka | 修复后 |
|---|---|---|---|
| `max_distance` | 1000.0 | 100.0 | 1000.0 |
| `space_downsample_leaf_size` | 0.2 | 0.5 | 0.2 |
| `map_resolution` | 0.2 | 0.5 | 0.2 |
| `satu_acc` | 3.0 (g) | 3.0 (m/s²) | 29.41995 (m/s²) |

`map_resolution` 直接决定 iVox 最近邻→平面拟合→整条轨迹，属于必须对齐的参数。

## 三、修复后的实测结果（116 s bag）

| 指标 | 结果 |
|---|---|
| 关键帧数 | 1157（baseline）vs 1157（asuka），首帧时间戳完全相同 |
| 帧时间戳一致度 | 1154/1157 帧相差 ≤ 1 µs（其余最大 336 µs，源于点时间量化） |
| 关键帧点云总点数 | 22,532,814 vs 22,532,816（差 2 点，75/1157 帧差 ±1 点） |
| 里程计位置误差 | 均值 5.3 mm，最大 31 mm，末帧 1.7 mm，无发散趋势（各十分位均值 4–10 mm 波动） |
| 里程计姿态误差 | 均值 4.4e-4 rad，最大 2.4e-3 rad |
| 里程计速度误差 | 均值 8.1e-2 m/s，末帧 1.5e-2 m/s |

对比前的状态：**asuka 根本跑不出结果**（2.2 节的死锁），无法比较。

## 四、剩余差异及原因（有意保留）

1. **点时间戳的表示**：asuka 的 `PointXYZIOffset::offset` 是 float 毫秒相对偏移，算法侧重建为
   `stamp + offset/1000`；上游用点字段的绝对 double 秒（`ts*1e-9`）。在 1.7e9 s 量级上 double 量化
   约 238 ns，float 毫秒约 7.6 ns，两者相差 ≤ ~0.5 µs。这会让"恰好落在边界上"的点在
   `timestamp >= last_timestamp_*` / `< imu.stamp` 的比较中翻转 —— 表现为 75 帧各差 ±1 个点，
   以及 3 帧的帧边界相差几百 µs。这是 32 字节点类型的固有精度，也是里程计毫米级差异的来源之一。
2. **加速度缩放的两层舍入**：上游一次乘法（`×9.81`），asuka 两次（`×9.80665` 再 `×(9.81/9.80665)`），
   相对差 ≤ 2 ULP。非线性滤波器会把末位差异放大为离散判定翻转（平面匹配接受/拒绝等），
   这正是毫米级状态差异的主要来源；差异有界、不随时间累积（表三）。
3. **关键帧点云坐标系**：asuka 存 IMU 系点云 + 帧末位姿；上游发布时把**每个点在消费时刻的位姿**
   烘焙进世界系点云。因此世界系点云逐点差异与帧内旋转量成正比：实测角速度 2.2–2.8 rad/s
   （一帧 12–16°）的帧，对应点半径方向角度差 p50 1.8–6.4°；近乎静止的帧只有 3 mm。
   这是表示差异，不是数据通路差异 —— 点集本身（帧内点数与逐点对应关系）一致。

## 五、未覆盖项与后续

* 本 bag 未触发 IMU 饱和（|acc| ≤ 1.4 g，|gyro| ≤ 2.63 rad/s），因此 2.5/2.6 的修复在本轮对比中
  不被覆盖；它们由代码逐行核对 + 上表统计判定。
* `extrinsic_est_en` 路径在 asuka 中仍为注释状态（上游配置默认 false），未验证。
* `reset()` / 重定位路径上游未在本轮触发。
* 执行器改动（2.1）影响所有前端算法（fastlio/batchlio/superlio/lightning）：它们现在收到与实时
  ROS 回调一致的逐条数据流，而不是成批数据；建议后续对它们各跑一次同类 A/B 回归
  （fastlio 已做冒烟：15 s 切片 142 帧、轨迹正常）。

## 六、受影响的既有文档

以下页面描述的是改动前的行为，需要据本页更新（`docs/odometry.md` 与
`docs/wiki/dataflow/02.md` 已加变更提示）：

* `docs/odometry.md`（点云预处理归属、dense/filtered）
* `docs/wiki/dataflow/01.md`、`02.md`、`03.md`（执行器批处理 → 逐条 FIFO）
* `docs/wiki/odometry/02.md`、`04.md`、`05.md`、`12.md`（smallpointlio 实现细节）
* `docs/wiki/ros/03.md`、`docs/index.md`、`docs/api.md`、`docs/point_type.md`（涉及 `offset`
  单位与预处理链路的描述）
* `docs/parameters.md`（smallpointlio 的参数默认值与 `imu_buffer_capacity` 已不再被读取）

