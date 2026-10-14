# Installation

## 依赖

| 依赖 | 说明 |
|---|---|
| ROS1 Noetic | `roscpp` `nodelet` `pluginlib` `pcl_ros` `pcl_conversions` `std_msgs` `sensor_msgs` `nav_msgs` `geometry_msgs` `tf2_ros` `tf2_eigen` `roslib` `rosbag` |
| GCC >= 9 | 需要 `<filesystem>` 与完整 C++17 支持 |
| CMake >= 3.0.2 | |
| Eigen3 / Boost / PCL | 系统包 |
| spdlog | 日志（`find_package(spdlog)`） |
| OpenCV + cv_bridge | 视觉输入、`PointXYZITRGB` 点类型与 fastlivo/srlivo（无条件依赖） |
| GTSAM | 仅 `BUILD_OPTIMIZATION_MODULES=ON` 时需要（`optimization_loam`） |
| glog / TBB | 仅优化模块与 `superlio` 需要 |

## 工作区布局

```
src/
├── asuka/        # 算法库包：asuka_core、六前端插件、算法扩展、config/、thirdparty/
├── asuka_ros/    # ROS 适配包：AsukaROS 适配层、rviz_viewer、apps 三入口、launch/
└── fastlio/      # 原版 FAST-LIO 参考实现（对照评测用，可选保留）
```

`asuka` 与 `asuka_ros` 均为 catkin 包；`fastlio` 仅作上游对照基准，不参与 asuka 运行时。

## 构建

```bash
cd /path/to/catkin_ws
catkin build asuka asuka_ros
```

`catkin` 配置变量（两包各自持有，**必须同值**）：

| 变量 | 默认 | 生效包 | 说明 |
|---|---|---|---|
| `BUILD_OPTIMIZATION_MODULES` | `OFF` | asuka | 是否编译 `optimization_loam` 后端扩展；`OFF` 时不查找、不链接 GTSAM |
| `BUILD_ODOMETRY_{FASTLIO,SMALLPOINTLIO,BATCHLIO,SUPERLIO,FASTLIVO,SRLIVO}` | `ON` | asuka | 每个里程计前端插件的独立编译开关（前端由 config_odometry 运行期 dlopen，关掉不影响核心） |

需要后端模块时：

```bash
catkin build asuka asuka_ros --cmake-args -DBUILD_OPTIMIZATION_MODULES=ON
```

!!! note "PCL_NO_PRECOMPILE"
    asuka 使用自定义点类型，CMake 已全局定义 `PCL_NO_PRECOMPILE`，使 `VoxelGrid` /
    `IterativeClosestPoint` 等 PCL 模板算法从头文件实现对自定义类型就地实例化。
    这也意味着编译比普通 ROS 包更慢、目标文件更大，属预期行为。

## 文档站点

```bash
pip3 install -r requirements.txt   # mkdocs + mkdocs-material
mkdocs build                       # 生成 site/
mkdocs serve                       # 本地预览 http://127.0.0.1:8000
```
