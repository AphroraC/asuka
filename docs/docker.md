# Docker

asuka 的开发与运行均在 docker 容器内进行；**宿主机不做任何写操作，也不安装任何依赖环境或开发工具**。

## 进入容器

```bash
docker exec -it dynamicx bash
```

容器内以 root 运行，工作空间挂载于 `/share/mapping_ws`：

```
/share/mapping_ws
├── src/asuka        # 算法库包（本项目主体）
├── src/asuka_ros    # ROS 适配包（入口 / rviz_viewer / launch）
├── src/nagisa,only  # 同仓其他算法包（感知滤除链 / GLIL 定位，与 asuka 无构建依赖）
├── test/...         # 各上游算法原始实现（迁移对照用）
└── build / devel    # catkin_tools 工作空间
```

## 容器内构建与运行

```bash
cd /share/mapping_ws
source /opt/ros/noetic/setup.bash
catkin build asuka asuka_ros --cmake-args -DBUILD_OPTIMIZATION_MODULES=ON
source devel/setup.bash
```

测试数据位于容器挂载目录：

| 数据 | 路径 |
|---|---|
| rosbag | `/share/rosbag/`：`mapping.bag`（livox PointCloud2，529 s）、`liantiao.bag`（livox PointCloud2，116 s）、`2026-09-10-01-03-00.bag`（robosense，104 s）、`hku.bag` / `garden.bag`（livox CustomMsg + 10 Hz 图像，LIVO 素材） |
| 地图输出 | `/share/pointclouds` |
| 日志 | `src/asuka/logs/`（由 `config.json` 的 `logging.logging_dir` 指定） |
