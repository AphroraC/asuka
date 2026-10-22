# Architecture

## 包结构

| 包 | 内容 |
|---|---|
| `asuka` | `asuka_core`（配置/日志/回调槽/执行器/插件加载）、七前端插件、`imu_prediction` 与优化扩展、`config/`、`thirdparty/`。无 ROS 依赖（catkin 环境下以无组件包形式导出，亦可独立 CMake 安装） |
| `asuka_ros` | `AsukaROS` 适配层、`rviz_viewer` 扩展、三个入口 app、launch、nodelet_plugins.xml。全仓唯一链接 catkin 的包 |

## 分层与线程模型

| 层 | 代码 | 线程 |
|---|---|---|
| ROS 入口 | `asuka_ros/apps/asuka_{rosnode,rosbag,nodelet}.cpp` | ROS spinner（nodelet 为独立 callback queue + 1 线程 AsyncSpinner） |
| ROS 层 | `asuka_ros/src/asuka_ros/asuka_ros.cpp` | spinner 线程：消息转换、时间/刻度校正、**点云预处理**、回绕检测 |
| 异步执行器 | `asuka/core/async_odometry_estimation.cpp` | 前端 worker：按传感器时间归并逐条取出事件 -> 发射输入槽 -> `insert_imu` / `insert_frame` / `insert_image` -> `process_once` 排空；事件级异常边界（log+skip），连续 10 次失败熔断有序停机（`is_stopped()` 可查） |
| 前端插件 | `asuka/odometry/*/` | 仅前端 worker 访问算法状态（缓冲入队加锁） |
| 扩展 | `imu_prediction` / 优化扩展在 `asuka`；`rviz_viewer` 在 `asuka_ros` | 各自线程（一律"回调只入队"） |

> 算法私有的 vendored 头（fastlio 的 esekfom+mtk+ikd_tree、batchlio 的 esekfom+mtk+ivox3d、
> smallpointlio 的 small_ivox、superlio/srlivo 的 tsl、zlio 的 nanoflann 等）已归位到各前端的
> `include/asuka/odometry/<前端>/` 下（含 `asuka_ikd_tree` 静态库源码）；
> `thirdparty/` 仅保留跨模块共享件（ankerl/livox_ros_driver 生成头/
> nlohmann/scancontext/sophus/vikit）。

`AsyncOdometryEstimation`：三条有界环形缓冲——IMU（1000）/ 帧（50）/ 图像（50），满则丢该流最旧；worker 按队首传感器时间三路归并逐条取出
（到达序，平局优先 imu -> image -> frame）、**每条先发射对应输入槽再喂给算法**，随后
排空 `process_once()`；`stop()` 置位并 join worker。

## 全局回调插槽

`CallbackSlot<Func>`（`utility/callback_slot.hpp`）为多播插槽：

- **copy-on-write 快照**：add/remove 在互斥锁下复制列表并原子发布新快照；发射端只做
  原子读，无锁、无分配、无 std::function 拷贝；
- 稳定整数 id，remove 留空洞不回收 id；
- 逐观察者 `try/catch`，异常经进程级 handler（`set_callback_exception_handler`，
  asuka_core 安装 spdlog 记录版）上报，不打断后续观察者与发射线程；
- 编译期禁止非平凡右值实参（防止首个观察者搬空参数）。

六个核心槽：`on_insert_imu` / `on_insert_frame` / `on_insert_image`
/ `on_new_frame` / `on_odometry_imu` / `on_odometry_opt`。输入槽由 `AsyncOdometryEstimation`
在 worker 线程上、喂算法之前统一发射——扩展观察到的输入流与所加载的前端算法无关；
`on_new_frame` 由各前端算法在 worker 线程发射；`on_odometry_imu` / `on_odometry_opt`
分别由 imu_prediction 与后端优化扩展在自己的线程上发射。**三输入槽 + `on_new_frame`
共用 worker 线程，交错顺序即算法程序序**（imu_prediction 锚定积分依赖此契约）。

**跨 .so 单实例**：插槽静态对象必须定义在 `asuka_core` 的 .cpp 中（`core/callbacks.cpp`）。
若定义在头文件内联，每个插件会各持一份，注册静默失效——asuka 已在头文件注释中显式记录。

## 生命周期

构造（`AsukaROS`）：进程级单实例守卫（第二实例直接 throw）-> ROS 参数解析 `config_path`（三级回退）-> `GlobalConfig` 单例 ->
`OdometryEstimation::load_configured()`（config/extension 加载基础设施均在 asuka 库）->
借出 `cloud_preprocess` -> 创建异步执行器 -> 读 ROS 配置 ->
`ExtensionModule::load_configured_modules()`。

析构：`async->stop()`（join worker，此后无槽发射源）-> 逐扩展 `stop()` ->
逐扩展 `at_exit(map_saving_path)`（逐扩展 try/catch 隔离，单个失败不阻断其余）->
成员析构（扩展析构中退订、里程计析构中清缓冲）。

## 插件加载与 ABI

- `utility/load_module.cpp`：`dlopen(RTLD_LAZY)`（默认 RTLD_LOCAL，多插件符号互不干扰），
  进程级句柄注册表，插件常驻、永不 `dlclose`；
- 加载失败分级：里程计加载失败 `load_configured()` 直接 throw（fail-fast）；扩展失败
  记 warning 后跳过；
- **ABI 无握手**：`KeyFrame` / 插槽布局编入每个 .so，头文件变更后必须全量重编所有插件；
  框架级 `-mcx16`（asuka 与 asuka_ros 两包各自保证）使 shared_ptr 原子操作在所有
  编译单元中一致；`PCL_NO_PRECOMPILE` 同理两包显式定义。

## 关键设计决策

| 决策 | 理由 |
|---|---|
| 32 B 自定义点类型 `PointXYZITRGB`（`curvature` 承载逐点毫秒时间，PCL 滤镜原生兼容） | 见[点云格式](point_type.md)；OpenCV 为无条件依赖，六个 LIO 前端零改动 |
| 预处理在 ROS spinner 线程 | 保持前端 worker 纯算法；代价是重预处理会占 spinner（当前预处理很轻） |
| KeyFrame::stamp 钉死扫描起始时刻 | stamp 是扫描身份（等于输入扫描 header 时刻）；载荷位姿通常指扫描末，二者有意解耦（契约见 `callbacks.hpp`） |
| 传感器外参随算法走（各 odometry config 的 `sensor` 块） | 每个算法对齐其上游基准外参，不存在共享块选择错误；camera 块同理（fastlivo） |
| 扩展参数集中于 config_extensions.json | 扩展与算法解耦：任何扩展对任何前端可用 |
| config/extension 加载基础设施在 asuka 库 | `load_configured()` / `load_configured_modules()` 让非 ROS 消费方与 AsukaROS 走同一条装载路径 |
| 输入槽发射在执行器、输出槽发射在前端/扩展 | 框架保证"喂给算法的每条数据同时可见于订阅者"；重活由扩展自行入队转移 |
| KeyFrame 点云逐帧新分配、以 shared_ptr 共享 | 无观察者深拷贝；算法发射后不得再写入该缓冲 |
| 存图为全量累积语义 | 各前端 save_map 均按其上游存图语义累积（fastlio/fastlivo = 上游 `pcl_wait_save` 逐帧追加），估计用的滑动局部图不作为地图导出 |
