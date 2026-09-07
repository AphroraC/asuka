# Docker

asuka 的开发与运行均在 docker 容器内进行；**宿主机不做任何写操作，也不安装任何依赖环境或开发工具**。

## 进入容器

```bash
docker exec -it dynamicx bash
```

容器内以 root 运行，工作空间挂载于 `/share/pilot_ws`：

```
/share/pilot_ws
├── src/asuka        # 本项目
├── src/test/...     # 各上游算法原始实现（迁移对照用）
├── build / devel    # catkin_tools 工作空间
└── AGENTS.md        # 环境约定
```

## 容器内构建与运行

```bash
cd /share/pilot_ws
source /opt/ros/noetic/setup.bash
catkin build asuka --cmake-args -DBUILD_OPTIMIZATION_MODULES=ON
source devel/setup.bash
```

测试数据位于容器挂载目录：

| 数据 | 路径 |
|---|---|
| rosbag | `/share/rosbag/mapping.bag`、`/share/rosbag/liantiao.bag` |
| 地图输出 | `/share/pointclouds` |
| 日志 | `src/asuka/logs/`（由 `config.json` 的 `logging.logging_dir` 指定） |
