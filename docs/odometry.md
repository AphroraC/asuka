# Odometry plugins

## 插件机制

每种前端算法编译为独立共享库（`libasuka_odometry_<name>.so`），导出 C 工厂函数：

```cpp
extern "C" asuka::OdometryEstimation* create_odometry_estimation();
```

`AsukaROS` 构造时按 `config_odometry` 的 `so_name` 经 `dlopen` 加载（进程级句柄注册表、
永不 `dlclose`），得到 `std::shared_ptr<asuka::OdometryEstimation>` 基类指针，
随后封装为 `AsyncOdometryEstimation` 异步执行器。

## 基类接口（asuka/core/odometry_estimation.hpp）

| 接口 | 说明 |
|---|---|
| `insert_imu(const ImuData::ConstPtr&)` | 追加 IMU（worker 线程调用） |
| `insert_frame(double stamp, const PointCloudT::ConstPtr&)` | 追加预处理后的帧 |
| `process_once()` | 处理一个同步单元；有输出返回 true |
| `workload()` | 待处理帧数（背压与退出判断依据） |
| `stop()` / `clear_buffers()` | 清空内部缓冲（**当前框架未调用 clear_buffers，见 FAQ**） |
| `save_map()` | 返回累积地图（调用前必须先停止 worker） |
| `cloud_preprocess()` | 返回算法专属的 `CloudPreprocess`，由 AsukaROS 持有并在 ROS 线程调用 |
| `requires_imu()` | 是否依赖 IMU |

关键帧输出：`process_once` 内部构造 `KeyFrame` 并调用 `Callbacks::on_new_frame`，
不做轮询导出。

## 内置算法

| 插件 | 上游 | 说明 |
|---|---|---|
| `libasuka_odometry_fastlio.so` | FAST-LIO2 | ikd-tree + IKFoM 迭代误差状态 EKF |
| `libasuka_odometry_batchlio.so` | Batch-LIO (Point-LIO 系) | 滑窗批量优化，iVox 局部地图 |
| `libasuka_odometry_smallpointlio.so` | Small Point-LIO | 逐点更新 ESKF + small iVox |
| `libasuka_odometry_superlio.so` | Super-LIO | Super voxel 流形 ESKF |
| `libasuka_odometry_lightning.so` | AAFasterLIO (Lightning) | iVox + ESKF，自有观测打包 |

每个算法自带 `ImuPreprocess` / `CloudPreprocess` 派生类（IMU 初始化与去畸变方式不同，
点云预处理逻辑取自各自上游），由算法构造函数创建、经 `cloud_preprocess()` 交给框架。

!!! warning "迁移一致性"
    各算法的目标行为与上游原始实现**完全一致**。重构点类型（48 B -> 32 B）后，
    相同输入下 fastlio 输出地图与旧构建逐字节一致；修改任何算法内部实现前请先建立
    同等强度的回归基线。

## 新增一种前端算法

1. 在 `include/asuka/odometry/<name>/` 与 `src/asuka/odometry/<name>/` 下实现
   `asuka::OdometryEstimation` 派生类（参数在构造函数经 `Config` 读入、直接作为成员变量）；
2. 提供 `create.cpp`：

   ```cpp
   extern "C" asuka::OdometryEstimation* create_odometry_estimation() {
     return new asuka::<name>::OdometryEstimation();
   }
   ```

3. `CMakeLists.txt` 添加 `add_library(asuka_odometry_<name> SHARED ...)` 并链接 `asuka_core`，
   加入 `install(TARGETS ...)`；
4. 新增 `config/odometry/config_<name>.json`，在 `config.json` 的 `config_odometry` 中启用。
