# Installation

## 依赖

| 依赖 | 说明 |
|---|---|
| ROS1 Noetic | `roscpp` `nodelet` `pcl_ros` `pcl_conversions` `std_msgs` `sensor_msgs` `nav_msgs` `geometry_msgs` `tf2_ros` `tf2_eigen` |
| GCC >= 9 | 需要 `<filesystem>` 与完整 C++17 支持 |
| CMake >= 3.0.2 | |
| Eigen3 / Boost / PCL | 系统包 |
| spdlog | 日志（`find_package(spdlog)`） |
| GTSAM | 仅 `BUILD_OPTIMIZATION_MODULES=ON` 时需要（`optimization_loam`） |
| glog / TBB | 仅优化模块与 `superlio` / `lightning` 需要 |

## 构建

```bash
cd /path/to/catkin_ws
catkin build asuka
```

`catkin` 配置变量：

| 变量 | 默认 | 说明 |
|---|---|---|
| `BUILD_OPTIMIZATION_MODULES` | `OFF` | 是否编译 `optimization_loam` / `optimization_miao` 后端扩展及其专属静态库（`asuka_miao`、`asuka_scancontext`）；`OFF` 时不查找、不链接 GTSAM |

需要后端模块时：

```bash
catkin build asuka --cmake-args -DBUILD_OPTIMIZATION_MODULES=ON
```

!!! note "PCL_NO_PRECOMPILE"
    asuka 使用自定义点类型 `asuka::PointXYZIOffset`，CMake 已全局定义 `PCL_NO_PRECOMPILE`，
    使 `VoxelGrid` / `IterativeClosestPoint` / `NormalDistributionsTransform` 等 PCL 模板算法
    从头文件实现对自定义类型就地实例化。这也意味着 asuka 的编译比普通 ROS 包更慢、目标文件更大，
    属预期行为。

## 文档站点

```bash
pip3 install -r requirements.txt   # mkdocs + mkdocs-material
mkdocs build                       # 生成 site/
mkdocs serve                       # 本地预览 http://127.0.0.1:8000
```
