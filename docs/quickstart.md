# Quick start

三种入口共享同一套配置与前端算法，按数据来源选择其一。

## 离线回放（rosbag）

```bash
roslaunch asuka rosbag.launch bag_path:=/share/rosbag/liantiao.bag
# 或直接运行（支持通配符与多包按序回放）
rosrun asuka asuka_rosbag /share/rosbag/liantiao.bag
```

- 自动发布 `/clock` 并周期打印回放速度；
- 带背压控制：前端积压（`workload > 10`）时暂停推帧，500 ms 无进展则放行；
- 回放结束后 `wait` 至前端空闲，自动保存地图后退出。

## 在线节点

```bash
roslaunch asuka rosnode.launch        # rviz:=false 关闭 rviz
```

## Nodelet

```bash
roslaunch asuka nodelet.launch        # 启动 nodelet manager 并加载 asuka/AsukaNodelet
```

Nodelet 使用独立的 `ros::CallbackQueue` 与单线程 `AsyncSpinner`，适合与传感器驱动同进程部署。

## 输出话题

话题由 `rviz_viewer` 扩展发布（见 `config_extensions.json` 的 `world_frame_id` / `imu_frame_id`，
默认 `camera_init` / `body`）：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/asuka/odometry` | `nav_msgs/Odometry` | 前端里程计（每关键帧） |
| `/asuka/odometry_imu` | `nav_msgs/Odometry` | IMU 插帧高频里程计（需加载 `imu_prediction`） |
| `/asuka/odometry_opt` | `nav_msgs/Odometry` | 后端优化后里程计（需加载优化扩展） |
| `/asuka/points` | `sensor_msgs/PointCloud2` | 当前帧去畸变点云（IMU 系） |
| `/asuka/world` | `sensor_msgs/PointCloud2` | 世界系点云 |

TF（`world -> imu`）默认由 `imu_prediction` 的高频流发布；未加载 `imu_prediction` 时由
`rviz_viewer` 逐关键帧发布（见 [FAQ](faq.md)）。

## 地图保存

进程退出前自动保存（`map_saving_path` 在 `config_ros.json`；是否产出地图由所选算法的
odometry json 控制，如 fastlio / batchlio / superlio 的 `mapping.enable_map_saving`，
lightning 始终导出其全局地图，smallpointlio 目前不产出地图）：

- 路径无扩展名 → 写入 `<path>/mapping_<时间戳>.pcd`；
- 路径带 `.pcd` 扩展名 → 按指定文件名写入；
- 保存格式为 `pcl::PointCloud<pcl::PointXYZI>` 二进制 PCD。
