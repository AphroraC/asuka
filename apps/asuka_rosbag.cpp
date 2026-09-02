#include <glob.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <asuka/asuka_ros.hpp>
#include <asuka/utility/config.hpp>
#include <asuka/utility/logging.hpp>

namespace {

class SpeedCounter {
public:
  SpeedCounter(const ros::Time& begin, const ros::Time& end)
  : begin_time(begin),
    end_time(end),
    last_real_time(std::chrono::high_resolution_clock::now()) {}

  void update(const ros::Time& stamp) {
    const auto now = std::chrono::high_resolution_clock::now();
    if (now - last_real_time < std::chrono::seconds(15)) {
      return;
    }

    if (last_sim_time.sec || last_sim_time.nsec) {
      const double real_sec = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_real_time).count() * 1e-9;
      const double sim_sec = (stamp - last_sim_time).toSec();
      const double playback_speed = sim_sec / std::max(1e-9, real_sec);

      const double current = (stamp - begin_time).toSec();
      const double duration = (end_time - begin_time).toSec();
      const double percentage = duration > 0.0 ? 100.0 * current / duration : 0.0;

      spdlog::info("playback speed: {:.3f}x  {:.2f}s/{:.2f}s ({:.2f}%)", playback_speed, current, duration, percentage);
    }

    last_sim_time = stamp;
    last_real_time = now;
  }

private:
  const ros::Time begin_time;
  const ros::Time end_time;
  ros::Time last_sim_time;
  std::chrono::high_resolution_clock::time_point last_real_time;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "asuka_rosbag");
  ros::NodeHandle nh("~");
  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1);

  asuka::AsukaROS asuka_ros(nh);
  spdlog::info("Starting Asuka rosbag player");

  const asuka::Config config(asuka::GlobalConfig::get_config_path("config_ros"));
  const std::string imu_topic = config.param<std::string>("ros", "imu_topic", "/livox/imu");
  const std::string lidar_topic = config.param<std::string>("ros", "lidar_topic", "/livox/lidar");
  const std::string map_saving_path = config.param<std::string>("ros", "map_saving_path", "");
  const std::vector<std::string> topics = {imu_topic, lidar_topic};

  spdlog::info("topics:");
  for (const auto& topic : topics) {
    spdlog::info("- {}", topic);
  }

  std::vector<std::string> bag_filenames;

  std::string param_bag_path;
  if (nh.getParam("bag_path", param_bag_path) && !param_bag_path.empty()) {
    bag_filenames.push_back(param_bag_path);
  }

  for (int i = 1; i < argc; ++i) {
    glob_t globbuf;
    if (glob(argv[i], 0, nullptr, &globbuf) == 0) {
      for (std::size_t j = 0; j < globbuf.gl_pathc; ++j) {
        bag_filenames.push_back(globbuf.gl_pathv[j]);
      }
    }
    globfree(&globbuf);
  }
  std::sort(bag_filenames.begin(), bag_filenames.end());

  if (bag_filenames.empty()) {
    spdlog::error("no input rosbag provided. Set ~bag_path param or pass as argv.");
    return 1;
  }

  spdlog::info("bag_filenames:");
  for (const auto& bag_filename : bag_filenames) {
    spdlog::info("- {}", bag_filename);
  }

  const auto read_bag = [&](const std::string& bag_filename) -> bool {
    spdlog::info("opening {}", bag_filename);
    rosbag::Bag bag(bag_filename, rosbag::bagmode::Read);
    if (!bag.isOpen()) {
      spdlog::error("failed to open {}", bag_filename);
      return false;
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    SpeedCounter speed_counter(view.getBeginTime(), view.getEndTime());

    for (const rosbag::MessageInstance& m : view) {
      if (!ros::ok()) {
        return false;
      }
      speed_counter.update(m.getTime());

      const std::string& topic = m.getTopic();

      if (topic == imu_topic) {
        const auto imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg) {
          asuka_ros.insert_imu(imu_msg);
        } else {
          spdlog::error("failed to instantiate IMU message");
        }
      } else if (topic == lidar_topic) {
        const auto points_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (points_msg) {
          auto last_progress = std::chrono::steady_clock::now();
          int last_workload = asuka_ros.workload();
          while (asuka_ros.workload() > 10) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            const int workload = asuka_ros.workload();
            if (workload < last_workload) {
              last_progress = std::chrono::steady_clock::now();
            }
            last_workload = workload;
            if (std::chrono::steady_clock::now() - last_progress > std::chrono::milliseconds(500)) {
              break;
            }
          }
          asuka_ros.insert_frame(points_msg);
        } else {
          spdlog::error("failed to instantiate PointCloud2 message");
        }
      }

      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock = m.getTime();
      clock_pub.publish(clock_msg);
      ros::spinOnce();
    }

    return true;
  };

  for (const auto& bag_filename : bag_filenames) {
    if (!read_bag(bag_filename)) {
      break;
    }
  }

  asuka_ros.wait(true);
  asuka_ros.save(map_saving_path);

  return 0;
}
