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
| `insert_image(const ImageData::ConstPtr&)` | 视觉输入（纯虚：LiDAR 前端显式空 override，fastlivo/srlivo 完整实现） |
| `process_once()` | 处理一个同步单元；有输出返回 true |
| `workload()` | 待处理帧数（背压与退出判断依据） |
| `stop()` | 清空内部缓冲（析构链路调用） |
| `save_map()` | 返回地图（调用前必须先停止 worker；统一受 `config_ros` 的 `ros.enable_map_saving` 门控）。各前端语义不同：fastlio / fastlivo 按上游 `pcl_wait_save` 逐帧追加全量去畸变扫描，batchlio / superlio 逐帧累加世界系点云，srlivo 为带色色图 |
| `cloud_preprocess()` | 返回算法专属的 `CloudPreprocess`，由 AsukaROS 持有并在 ROS 线程调用（所有权仍在算法，AsukaROS 只借出共享指针；算法本身由 `AsyncOdometryEstimation` 独占持有，AsukaROS 不再单独持有算法指针） |

关键帧输出：`process_once` 内部构造 `KeyFrame` 并调用 `Callbacks::on_new_frame`，
不做轮询导出。**输入槽 `on_insert_imu` / `on_insert_frame` 由 `AsyncOdometryEstimation`
在喂算法之前统一发射，算法不应（也无需）自行触发**——契约见 `core/callbacks.hpp` 注释。

`KeyFrame::stamp` 全前端统一为**产生该关键帧的传感器扫描的起始时刻**（扫描 header
时刻；Point-LIO 类事件驱动前端取关键帧窗口首样本时刻）。注意载荷位姿/速度/偏置是
估计器当前状态，通常指扫描末——消费者不得假设 `stamp` 与位姿时刻重合。

## 内置算法

| 插件 | 上游 | 说明 |
|---|---|---|
| `libasuka_odometry_fastlio.so` | FAST-LIO2 | ikd-tree + IKFoM 迭代误差状态 EKF |
| `libasuka_odometry_batchlio.so` | Batch-LIO (Point-LIO 系) | 滑窗批量优化（output/output+IMU 量测双模式），iVox 局部地图；output 模式（上游默认）曾因移植缺陷发散，2026-09-26 根因修复后为推荐模式（faq 已知问题 1） |
| `libasuka_odometry_superlio.so` | Super-LIO | Super voxel 流形 ESKF |
| `libasuka_odometry_fastlivo.so` | FAST-LIVO2 | 稀疏体素八叉树 LIO + 光度 VIO |
| `libasuka_odometry_srlivo.so` | sr_livo | CT-ICP 连续点流里程计 + R3LIVE 式视觉渲染/在线相机标定 |
| `libasuka_odometry_zlio.so` | ZLIO (TDT z-localization) | 15 维半直积误差状态滤波 + 连续点流自适应观测窗口（无扫描概念） |
| livox CustomMsg 输入 | — | `lidar_type: "livox_custom"`：`asuka::LivoxCustomPoint`（驱动逐点 offset_time ns）+ `CloudPreprocess` 第三重载；**六前端均已实现** CustomMsg 处理（fastlivo 为上游 avia_handler 直移植，其余五前端为消息格式适配——抽稀/去重等仍在各自算法内进行） |

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
- **配置**：`config/odometry/config_fastlivo.json` 块结构与上游 yaml 对齐；
  `slam_mode` 显式参数（`ONLY_LIO`/`LIVO`，LIVO 无图像时按上游语义阻塞等待）。
  lidar→IMU 外参取本前端配置的 `sensor` 块；LIVO 模式同时读本配置的 `camera` 块
  （`distortion_model: plumb_bob→Pinhole`、`equidistant→Equidistant`，lidar→camera 外参由
  `T_imu_camera ∘ T_imu_lidar` 推导）。
- **零 using-directives / 零类型隐藏别名**：全仓（含 thirdparty vendored 代码）
  不使用 `using namespace`，也不保留 `V3D`/`M3D`/`Vec3d`/`Quat`/`Scalar` 这类
  Eigen/Sophus 类型隐藏别名——所有类型声明显式写 `Eigen::`/`Sophus::`。
  清理面：fastlivo 移植层（V2F/V2D/V3D/M3D/V3F/M3F + MD/VD/MF/VF 宏）、
  batchlio common.hpp、superlio alias.hpp（V*/M*/Quat/Scalar 全家族 ~440 处）、batchlio eigen_types.h
  （原 thirdparty/faster_ivox）、vikit、ikd_tree、esekfom（fastlio/pointlio 两份）、
  scancontext。例外（均为非污染结构，保留并说明）：模板依赖的成员别名
  （如 pointlio mtk 内部 typedef、
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

## srlivo（sr_livo 前端）

上游 `test/sr_livo`（COIN-LIO/CT-ICP 谱系 + R3LIVE 视觉渲染分支），单节点 `livo_node`。
移植为 `asuka::srlivo::OdometryEstimation`（第六前端）。

- **数据流**：三路输入经执行器进入插件内 `boost::circular_buffer` 连续点流/IMU/图像环形缓冲；
  `process_once()` 驱动上游 `getMeasurements()` 的**图像锚定切窗**状态机——窗界跟随图像时刻，
  图像过旧（点流/IMU 已越过）即丢弃，图像稀疏时切定长纯 LIO 窗（无图像不产出窗口，
  上游没有 ONLY_LIO 模式，本移植亦不提供）。
- **算法**：17 维手写 iEKF（p/q/v/ba/bg + S2 流形重力），CT-ICP 体素邻域点到面迭代；
  rendering 窗额外执行 R3LIVE 式 LK 光流跟踪 + 色图渲染 + 11 维在线相机内参/外参精细化。
- **输入**：三格式全支持。`"livox_custom"` / `"robosense"`（逐点 ns 时间戳）与 `"livox"`
  （逐点绝对时间戳归约到扫描时基）——livox 两路同为 r3live 语义滤波：line/IS_VALID/x>0.7/
  tag 尘埃/相邻去重，二段抽稀 + blind 球（livox 路径与 custom 路径经合成数据逐点等价验证）。
- **输出**：每个测量窗发射 `on_new_frame`（IMU 系去畸变点云，rendering 窗逐点投影取真实 rgb，
  LIO 窗 rgb=0）；`save_map()` 输出色图（rgbPoint 累计色，`pub_point_minimum_views` 过滤，
  受 config_ros `enable_map_saving` 门控）。
- **与上游的有意差异**：Ceres 假依赖删除（全码零使用）；发布/TF/pose.txt 落盘全删
  （rviz_viewer/imu_prediction 接管）；`time_diff_enable` 由 config_ros `imu_time_offset` 替代；
  findFundamentalMat 空集守卫（上游在 LK 全失败时会抛 OpenCV 异常）；有界环形缓冲
  （全局 parity 例外）；tsl robin_map 为 srlivo 目录私有 vendor（与 superlio 版本有分叉）。
- **配置**：`config/odometry/config_srlivo.json`（r3live.yaml 全量转写：preprocess/imu/camera
  含在线标定初值与外参/sensor/odometry_options/icp_options/map_options）。
- **验证**：hku.bag LIVO 全程 EXIT=0（1.58x、1.25M 点彩色图、rgb 覆盖 85.7%）、garden.bag
  EXIT=0（752k 点）、双跑 PCD md5 一致、
  `nm -D` 零 ros 符号。

## zlio（ZLIO 前端）

上游 `test/z-localization`（TDT 哨兵定位开源，RM2026），其前端核心库 libzlio（约 7.5k 行）
本身零 ROS 依赖，ROS2 只在上游 apps 层——是历次迁移中核心/ROS 分离最彻底的上游。

- **算法**：SE2(3) 半直积 15 维误差状态（姿态/速度无噪声误差精确线性，参考文献
  Equivariant Symmetries for Inertial Navigation Systems）+ 非静止初始化（多帧共面约束与
  IMU 预积分同窗口联合优化）+ DLIO 式分段连续轨迹逐点去畸变 + 点矩统计量体素八叉平面
  地图（增量更新、达限后释放统计量）+ 平面共享误差相关性 + 时空自适应（窗口间隔
  0.03-0.12 s 按运动激励调度、PD 控制采样体素）。
- **输入契约**：连续点流——上游仅消费带逐点时间的点云，无 intensity 使用（内部点
  xyz+ns+id）；量程过滤在算法内部。asuka 侧 CloudPreprocess 三重载为纯格式转换
  （curvature ms 时间基准），不添加任何上游没有的过滤。
- **KeyFrame 契约**：按观测窗口（而非扫描）节奏发射；`stamp` 取窗口起点（zlio 例外
  条款，见 `callbacks.hpp`）；位姿/速度/双偏置为窗口末端后验（15 维全填）；云经
  `odometry.output_resolution` 门控导出（默认 0.1 m，0 关闭）、intensity 恒 0；
  FrameResult 的 15 维协方差未进入 asuka 契约（丢弃）。
- **save_map**：上游无地图导出，适配层按全量累积语义实现（窗口末端后验转世界系逐帧
  追加）。
- **配置**：`config_zlio.json` 键结构与上游 lio.yaml 同构（默认值=TDT 实机值）；
  32 线大场景（robosense）需调大 `map.maximum_roots`（默认 1e6 为 Mid-360 基准，超限
  抛 "moment map exceeds root resource limit" 并由 worker 熔断停机）。
- **有意偏差**：yaml-cpp 读取层重写为 asuka 配置系统（校验层逐字保留）；nanoflann
  1.5+ 需显式 buildIndex（上游所用旧版构造即建，已补一行版本适配）；上游的高频位姿
  预测/暖重置/输入恢复未迁移（imu_prediction 扩展覆盖前者，后两者待 asuka 基类演进）。
  多线程（关联 2 线程/启动优化 5 线程）存在约 1e-10 的上游固有调度非确定性。
- **验证**：直链对拍（原版 vs 移植版同合成房间场景，单线程 116 帧全字段逐位一致；
  多线程差异经自跑复现证实为上游并行调度非确定）；三格式端到端全绿（liantiao
  11,381,390 / hku 10,780,201 / robosense 9,011,029 存图点，零 worker 异常、EXIT=0）；
  双跑 PCD md5 一致；树内 gtest（state/input/initialization 三套上游测试）3/3 通过。

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
4. 新增 `config/odometry/config_<name>.json`，在 `config.json` 的 `config_odometry` 中启用。
