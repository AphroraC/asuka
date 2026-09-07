# Extending Asuka

扩展模块（`ExtensionModule`）是运行时 `dlopen` 的共享库，用于消费前端状态或注入自己的
处理线程：后端优化、IMU 高频预测、可视化都是扩展。

## 记号与数据类型

位姿记号 `T_world_imu` 表示 IMU 系到世界系的变换（`p_world = T_world_imu * p_imu`）。
扩展模块收到的核心数据结构：

```cpp
struct KeyFrame {
  long id;                                  // 前端关键帧序号（预测/优化帧为 -1）
  double stamp;                             // 帧时间戳（首点或末点，由算法决定）
  FrameId frame_id;                         // FrameId::IMU
  Eigen::Isometry3d T_world_imu;            // 位姿
  Eigen::Vector3d v_world_imu;              // 世界系速度
  Eigen::Matrix<double, 6, 1> imu_bias;     // [gyro_bias, acc_bias]
  PointCloudT::Ptr cloud_imu;               // 去畸变点云（IMU 系，帧独有副本）
};
```

`on_odometry_imu` / `on_odometry_opt` 发出的帧仅填充 `stamp` / `T_world_imu` /
`v_world_imu`（`id = -1`、无点云），用于轨迹输出。

## 全局回调插槽

`asuka::Callbacks`（定义于 `asuka_core` 的 .cpp，保证跨 .so 单实例）提供五个插槽：

| 插槽 | 签名 | 发射时机 |
|---|---|---|
| `on_insert_imu` | `(const ImuData::ConstPtr&)` | 前端 worker 喂入每个 IMU 样本时 |
| `on_insert_frame` | `(double, const PointCloudT::ConstPtr&)` | 喂入每帧点云时 |
| `on_new_frame` | `(const KeyFrame::ConstPtr&)` | 前端产出关键帧时 |
| `on_odometry_imu` | `(const KeyFrame::ConstPtr&)` | imu_prediction 产出高频预测时 |
| `on_odometry_opt` | `(const KeyFrame::ConstPtr&)` | 后端产出优化位姿时 |

`CallbackSlot` 采用 copy-on-write 快照：发射路径无锁无分配，`add()` 返回的整数 id 用于
`remove()`；观察者异常被逐个捕获并记录，不会中断发射线程。**回调在发射者线程同步执行**
（通常是前端 worker），重活请入队到自己的线程——内置扩展均遵循"回调只入队"约定
（rviz_viewer 是历史例外，正逐步对齐）。

## 编写扩展模块

```cpp
// my_extension.cpp
#include <asuka/core/callbacks.hpp>
#include <asuka/utility/extension_module.hpp>

namespace asuka {

class MyExtension : public ExtensionModule {
public:
  MyExtension() {
    logger = create_module_logger("myext");
    const Config config(GlobalConfig::get_config_path("config_extensions"));
    param_ = config.param<double>("my_extension", "param", 1.0);

    id_ = Callbacks::on_new_frame.add(
        [this](const KeyFrame::ConstPtr& frame) { on_new_frame(frame); });
    // 自己开线程的模块：在这里启动
  }

  ~MyExtension() override {
    Callbacks::on_new_frame.remove(id_);   // 析构时退订
  }

  void stop() override {                   // 必须：join 自有线程，幂等
    // if (thread_.joinable()) thread_.join();
  }

private:
  void on_new_frame(const KeyFrame::ConstPtr& frame) {
    // 只做轻活 / 入队
  }

  int id_{-1};
  double param_{1.0};
  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace asuka

extern "C" asuka::ExtensionModule* create_extension_module() {
  return new asuka::MyExtension();
}
```

CMake：

```cmake
add_library(asuka_my_extension SHARED my_extension.cpp)
target_link_libraries(asuka_my_extension asuka_core)
# 并加入 install(TARGETS ...)
```

在 `config/config_extensions.json` 中启用：

```json
{
  "extensions": {
    "loading_extension_modules": [
      "libasuka_my_extension.so"
    ]
  },
  "my_extension": { "param": 1.0 }
}
```

!!! warning "生命周期与线程安全"
    框架析构顺序为：停止前端 worker -> 逐个调用扩展的 `stop()` -> `at_exit()` ->
    析构（析构中退订回调）。**扩展必须在自己的 `stop()` 中 join 所有会发射回调或访问
    共享状态的线程**（默认实现为空，注意覆写）；`stop()` 之后不得再有槽发射。

!!! note "不用动态加载时"
    也可以像 `apps/` 一样直接在入口代码中持有模块实例，绕过 `dlopen`；
    但推荐保持扩展形态以维持"核心零改动"。
