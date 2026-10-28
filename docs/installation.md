# Installation

## 依赖

| 依赖 | 说明 |
|---|---|
| ROS1 Noetic | `roscpp` `nodelet` `pluginlib` `pcl_ros` `pcl_conversions` `std_msgs` `sensor_msgs` `nav_msgs` `geometry_msgs` `tf2_ros` `tf2_eigen` `roslib` `rosbag` |
| GCC >= 9 | 需要 `<filesystem>` 与完整 C++17 支持 |
| CMake >= 3.0.2 | |
| Eigen3 / Boost / PCL | 系统包 |
| spdlog | 日志（`find_package(spdlog)`） |
| OpenCV + cv_bridge | 视觉输入、`PointXYZITRGB` 点类型与 fastlivo（无条件依赖） |

## 工作区布局

```
src/
├── asuka/        # 算法库包：asuka_core、三前端插件、config/、thirdparty/
└── asuka_ros/    # ROS 适配包：AsukaROS 适配层、rviz_viewer、imu_prediction、apps 三入口、launch/
```

`asuka` 与 `asuka_ros` 均为 catkin 包；各上游原始实现位于仓库 `test/` 下（仅作迁移
对照基准，不参与 asuka 运行时）。

## 构建

```bash
cd /path/to/catkin_ws
catkin build asuka asuka_ros
```

`catkin` 配置变量（两包各自持有，**必须同值**）：

| 变量 | 默认 | 生效包 | 说明 |
|---|---|---|---|
| `BUILD_ODOMETRY_{FASTLIO,SUPERLIO,FASTLIVO}` | `ON` | asuka | 每个里程计前端插件的独立编译开关（前端由 config_odometry 运行期 dlopen，关掉不影响核心） |

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
