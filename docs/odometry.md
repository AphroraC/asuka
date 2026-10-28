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
| `insert_image(const ImageData::ConstPtr&)` | 视觉输入（纯虚：LiDAR 前端显式空 override，fastlivo 完整实现） |
| `process_once()` | 处理一个同步单元；有输出返回 true |
| `workload()` | 待处理帧数（背压与退出判断依据） |
| `stop()` | 清空内部缓冲（析构链路调用） |
| `save_map()` | 返回地图（调用前必须先停止 worker；统一受 `config_ros` 的 `ros.enable_map_saving` 门控）。各前端语义不同：fastlio / fastlivo 按上游 `pcl_wait_save` 逐帧追加全量去畸变扫描，superlio 逐帧累加世界系点云 |
| `cloud_preprocess()` | 返回算法专属的 `CloudPreprocess`，由 AsukaROS 持有并在 ROS 线程调用（所有权仍在算法，AsukaROS 只借出共享指针；算法本身由 `AsyncOdometryEstimation` 独占持有，AsukaROS 不再单独持有算法指针） |

关键帧输出：`process_once` 内部构造 `KeyFrame` 并调用 `Callbacks::on_new_frame`，
不做轮询导出。**输入槽 `on_insert_imu` / `on_insert_frame` 由 `AsyncOdometryEstimation`
在喂算法之前统一发射，算法不应（也无需）自行触发**——契约见 `core/callbacks.hpp` 注释。

`KeyFrame::stamp` 全前端统一为**产生该关键帧的传感器数据的起始时刻**（扫描 header
时刻）。注意载荷位姿/速度/偏置是估计器当前状态，通常指扫描末——消费者不得假设
`stamp` 与位姿时刻重合。

## 内置算法

| 插件 | 上游 | 说明 |
|---|---|---|
| `libasuka_odometry_fastlio.so` | FAST-LIO2 | ikd-tree + IKFoM 迭代误差状态 EKF |
| `libasuka_odometry_superlio.so` | Super-LIO | Super voxel 流形 ESKF |
| `libasuka_odometry_fastlivo.so` | FAST-LIVO2 | 稀疏体素八叉树 LIO + 光度 VIO |
| livox CustomMsg 输入 | — | `lidar_type: "livox_custom"`：`asuka::LivoxCustomPoint`（驱动逐点 offset_time ns）+ `CloudPreprocess` 第三重载；**三前端均已实现** CustomMsg 处理（fastlivo 为上游 avia_handler 直移植，其余两前端为消息格式适配——抽稀/去重等仍在各自算法内进行） |

每个算法自带 `ImuPreprocess` / `CloudPreprocess` 派生类（IMU 初始化与去畸变方式不同，
点云预处理逻辑取自各自上游），由算法构造函数创建、经 `cloud_preprocess()` 交给框架。

## fastlivo（FAST-LIVO2 前端）

`asuka::fastlivo::OdometryEstimation` 是 FAST-LIVO2 `LIVMapper` 算法核的 ROS-free 移植：
`sync_packages` 状态机（ONLY_LIO / LIVO 两模式，上游 ONLY_LO 不迁）+ FAST-LIO 式 IMU 传播
与逐点去畸变（`ImuPreprocess`）+ 稀疏体素八叉树平面地图与迭代 EKF（`voxel_map`）
+ 视觉稀疏子地图光度直法 EKF（`vio`，含逆曝光估计）。上游单线程 5000Hz 轮询循环由
asuka 的事件驱动 `process_once()` 等价替代。

- **构建**：整个插件（含 `asuka_fastlivo_basic` 静态辅助库）默认随
  `BUILD_ODOMETRY_FASTLIVO` 构建；`PointT` 为 `PointXYZITRGB`，LIO 帧 rgb 保持 0，
  VIO 更新后按上游投影取色逻辑给保存地图上色。
- **vendor**：`thirdparty/vikit`（相机模型最小集）。Sophus 使用既有共享的
  `thirdparty/sophus`（模板版）——`common_lib.hpp` 内
  `using SE3 = Sophus::SE3d` 让移植代码签名保持逐字，`rotation_matrix()` 更名为
  `rotationMatrix()`；vendored vikut 头的 `sophus/se3.h` include 已切到 se3.hpp，
  裸 `Matrix` 已加 Eigen 限定（模板 Sophus 的 `Sophus::Matrix` 别名会与
  `using namespace Eigen` 二义）。
- **配置**：`config/modules/config_fastlivo.json` 块结构与上游 yaml 对齐；
  `slam_mode` 显式参数（`ONLY_LIO`/`LIVO`，LIVO 无图像时按上游语义阻塞等待）。
  lidar→IMU 外参取本前端配置的 `sensor` 块；LIVO 模式同时读本配置的 `camera` 块
  （`distortion_model: plumb_bob→Pinhole`、`equidistant→Equidistant`，lidar→camera 外参由
  `T_imu_camera ∘ T_imu_lidar` 推导）。
- **零 using-directives / 零类型隐藏别名**：全仓（含 thirdparty vendored 代码）
  不使用 `using namespace`，也不保留 `V3D`/`M3D`/`Vec3d`/`Quat`/`Scalar` 这类
  Eigen/Sophus 类型隐藏别名——所有类型声明显式写 `Eigen::`/`Sophus::`。
  清理面：fastlivo 移植层（V2F/V2D/V3D/M3D/V3F/M3F + MD/VD/MF/VF 宏）、
  superlio alias.hpp（V*/M*/Quat/Scalar 全家族 ~440 处）、vikit、ikd_tree、
  esekfom 与 mtk（fastlio vendored）。例外（均为非污染结构，保留并说明）：模板依赖的成员别名
  （如 mtk 内部 typedef、
  Sophus 库自身 `Sophus::Vector/Matrix` 模板别名 API——使用处均带限定）；
  领域类型别名（`PointT`/`PointCloudT`/`IdType` 等，非 Eigen/Sophus 隐藏）。
- **队列**：输入队列一律 `boost::circular_buffer`（容量在 `odometry` 块）；
  `workload()` 仅统计 lidar 队列深度（IMU/图像不参与 rosbag 背压阈值）。
- **on_new_frame 节奏**：每次 EKF 更新发射一次——handleLIO 收敛后一帧 + handleVIO 一帧；
  同一扫描的 LIO 帧与其后 VIO 帧 `cloud_imu` 共享同一份 immutable 去畸变点云（逐扫描新分配）。
- **save_map**：上游 `pcl_wait_save` 语义逐帧追加（无去重）；ONLY_LIO 累积世界点，
  LIVO 在 VIO 更新时累积投影取色的世界点（`dense_map_en` 控制稠密/降采样输入）；
  仍受 `ros.enable_map_saving` 门控。
- **不迁**：feature_extract 分支（上游默认关）、Forward_without_imu/ONLY_LO、
  publish_*/tf/mavros/path、imu_prop 250Hz 外推（由 imu_prediction 扩展接管）、
  colmap/evo 文件输出、pcd 分段双文件、plot/imshow 调试路径。

### 参数对齐与验证状态

- `config_fastlivo.json` 当前值 = 上游 `test/FAST-LIVO2/config/avia.yaml`
  （img_time_offset 0.1、blind 0.8、point_filter_num 1、filter_size_surf 0.1、
  imu_int_frame 30、gyr/acc_cov 0.3/0.5、lio 体素参数 max_layer 2 / dept_err 0.02 /
  beam_err 0.05 / min_eigen_value 0.0025、patch_pyrimid_level 4、inv_expo_cov 0.1、
  blind_rgb_points 0、dense_map_en true）；`config_fastlivo.json` camera 块 = camera_pinhole.yaml
  （1280x1024、scale 0.5、plumb_bob d0..d3，T_imu_camera 由上游 Rcl/Pcl+extrinsic 推导），
  `sensor` 块外参同化为 [0.04165,0.02326,-0.0284]（注意：mapping.bag 是另一台 Avia，
  其原外参 [-0.011,-0.02329,0.04412] 见 git/历史记录）。
- **ONLY_LIO**：mapping.bag 6.1x / 9,471,361 点；robosense bag 506,896 点；
  上游对照（秒制转换 bag 夹具）终点位姿差 2.7cm（准静态退化序列）。
- **LIVO（hku.bag：CustomMsg 雷达 + 10Hz 图像）**：LIO+VIO 交替更新、视觉地图增长、
  存图 11.36M 点且 99% 以上点带真实 rgb；连续 6+ 次运行 0 崩溃、存图点数确定性一致。
  修复的两个实现层缺陷：① lidar→camera 外参经 `Matrix3d::data()`（列主序）与
  `MAT_FROM_ARRAY`（行主序）互转被隐形转置；② `updateState` 深层金字塔的 du/dv 梯度
  采样足迹（±(half+1)·scale）大于检索阶段 border，EKF 迭代把点推向边缘时会越界读
  图像缓冲（上游 UB），已加边界守卫跳过越界采样。
- **rgb 输出通路**（LIVO 模式）：VIO 更新时对 KeyFrame 云做投影取色的**新鲜副本**
  （LIO 帧已共享不可变；且 LIVO 下 LIO 更新不发云——对齐上游"彩色云仅随 VIO 更新发布"
  的可见行为）；保存 PCD 为 `x y z intensity rgb` 且 99%+ 点带真实 rgb
  （hku.bag 实测 11.36M 点，视场外点如实不着色、约 60% 在视场内）。注意 rviz 话题
  （/asuka/points、/asuka/world）固定按 `pcl::PointXYZI` 四字段发布、不带 rgb——
  rgb 仅体现在保存的 PCD。
  ONLY_LIO 模式无图像可着色，rgb=0 与上游强度输出语义一致。

- **garden.bag** 同为 CustomMsg+图像素材，可作扩展回归基线。


!!! warning "迁移一致性"
    各算法的目标行为与上游原始实现**完全一致**。重构点类型（48 B -> 32 B）后，
    相同输入下 fastlio 输出地图与旧构建逐字节一致；修改任何算法内部实现前请先建立
    同等强度的回归基线。

## 新增一种前端算法

1. 在 `include/asuka/modules/<name>/` 与 `src/asuka/modules/<name>/` 下实现
   `asuka::OdometryEstimation` 派生类（参数在构造函数经 `Config` 读入，集中在 `asuka::xxx::OdometryEstimationParam param` 成员中）；
2. 提供 `create.cpp`：

   ```cpp
   extern "C" asuka::OdometryEstimation* create_odometry_estimation() {
     return new asuka::<name>::OdometryEstimation();
   }
   ```

3. `CMakeLists.txt` 添加 `add_library(asuka_odometry_<name> SHARED ...)` 并链接 `asuka_core`，
   加入 `install(TARGETS ...)`；
4. 新增 `config/modules/config_<name>.json`，在 `config.json` 的 `config_odometry` 中启用。
