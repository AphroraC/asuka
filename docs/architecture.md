# Architecture

## 分层与线程模型

| 层 | 代码 | 线程 |
|---|---|---|
| ROS 入口 | `apps/asuka_{rosnode,rosbag,nodelet}.cpp` | ROS spinner（nodelet 为独立 callback queue + 1 线程 AsyncSpinner） |
| ROS 层 | `asuka_ros.cpp` | spinner 线程：消息转换、时间/刻度校正、**点云预处理**、回绕检测 |
| 异步执行器 | `core/async_odometry_estimation.cpp` | 前端 worker：批量倒队 -> `insert_imu` / `insert_frame` -> `process_once` 循环 |
| 前端插件 | `odometry/*/` | 仅前端 worker 访问算法状态（缓冲入队加锁） |
| 扩展 | `modules/*.cpp` | 各自线程（imu_prediction / optimization_* 为"回调只入队"模式） |

`AsyncOdometryEstimation`：IMU 队列 1000、帧队列 50（环形，满则丢最旧），每轮批量取出、
先 IMU 后帧喂入，随后排空 `process_once()`；`stop()` 置位并 join worker。

## 全局回调插槽

`CallbackSlot<Func>`（`utility/callback_slot.hpp`）为多播插槽：

- **copy-on-write 快照**：add/remove 在互斥锁下复制列表并原子发布新快照；发射端只做
  原子读，无锁、无分配、无 std::function 拷贝；
- 稳定整数 id，remove 留空洞不回收 id；
- 逐观察者 `try/catch`，异常经进程级 handler（`set_callback_exception_handler`，
  asuka_core 安装 spdlog 记录版）上报，不打断后续观察者与发射线程；
- 编译期禁止非平凡右值实参（防止首个观察者搬空参数）。

**跨 .so 单实例**：插槽静态对象必须定义在 `asuka_core` 的 .cpp 中
（`core/callbacks.cpp`）。若定义在头文件内联，每个插件会各持一份，注册静默失效——
这是插件式框架的经典坑，asuka 已在头文件注释中显式记录。

## 生命周期

构造（`AsukaROS`）：`GlobalConfig` 单例 -> 加载里程计插件 -> 取 `cloud_preprocess` ->
创建异步执行器 -> 读 ROS 配置 -> 加载扩展。

析构：`async->stop()`（join worker，此后无槽发射源）-> 逐扩展 `stop()` ->
逐扩展 `at_exit(map_saving_path)` -> 成员析构（扩展析构中退订、里程计析构中清缓冲）。

## 插件加载与 ABI

- `utility/load_module.cpp`：`dlopen(RTLD_LAZY)`（默认 RTLD_LOCAL，多插件符号互不干扰），
  进程级句柄注册表，插件常驻、永不 `dlclose`；
- 建议的加载失败策略为 fail-fast（RTLD_NOW 更利于尽早暴露缺符号）；
- **ABI 无握手**：`KeyFrame` / 插槽布局编入每个 .so，头文件变更后必须全量重编所有插件；
  框架级 `-mcx16` 保证 shared_ptr 原子操作在所有编译单元中一致（见 `CMakeLists.txt`）。

## 关键设计决策

| 决策 | 理由 |
|---|---|
| 32 B 自定义点类型 | 见 [点云格式](point_type.md)；重构后输出与旧实现逐位一致 |
| 预处理在 ROS spinner 线程 | 保持前端 worker 纯算法；代价是重预处理会占 spinner（当前预处理很轻） |
| 帧级 stamp 由算法决定 | 去畸变与滑窗语义属算法内部事务，框架不越权 |
| 扩展参数集中于 config_extensions.json | 扩展与算法解耦：任何扩展对任何前端可用 |
| 插槽发射在 worker 线程同步直调 | 零拷贝零延迟；重活由扩展自行入队转移 |
