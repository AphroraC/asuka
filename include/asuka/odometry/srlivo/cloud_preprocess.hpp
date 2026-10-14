#pragma once
// srlivo cloud preprocessing: port of sr_livo include/cloudProcessing.h for
// the two sensor paths asuka supports for this frontend. The upstream
// time_unit knob is dropped: asuka fixes the per-point time contract at
// nanoseconds (livox CustomMsg offset_time, robosense per-point timestamp),
// so the handler emits PointT offsets in milliseconds directly. The
// velodyne/ouster handlers and the commented fast-lio branch of the upstream
// are not migrated.

#include <memory>

#include <spdlog/spdlog.h>

#include <asuka/core/cloud_preprocess.hpp>
#include <asuka/core/types.hpp>

namespace asuka {
namespace srlivo {

class CloudPreprocess : public asuka::CloudPreprocess {
public:
  CloudPreprocess();

  // r3live livox semantics: skip the first raw point, line/IS_VALID/x>0.7
  // filtering, tag dust rejection on far points, 1e-7 adjacent dedup, then a
  // second pass applying point_filter_num and the blind sphere.
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxCustomPoint>::ConstPtr &raw) override;

  // upstream robosenseHandler: per-point timestamps (ns), sort, offset
  // relative to the first point, blind sphere, point_filter_num, and the
  // cross-scan last_end_time gate.
  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<RobosensePoint>::ConstPtr &raw) override;

  PointCloudT::Ptr preprocess(double stamp, const pcl::PointCloud<LivoxPoint>::ConstPtr &raw) override;

  void set_num_scans(int para) { num_scans = para; }
  void set_scan_rate(int para) {
    scan_rate = para;
    time_interval_sweep = 1.0 / static_cast<double>(scan_rate);
  }
  void set_blind(double para) { blind = para; }
  void set_point_filter_num(int para) { point_filter_num = para; }
  double get_sweep_interval() const { return time_interval_sweep; }

private:
  int num_scans{6};
  int scan_rate{10};
  double time_interval_sweep{0.1};
  double blind{0.1};
  int point_filter_num{1};

  double last_end_time{-1.0};
  double last_time_lidar{-1.0};

  std::shared_ptr<spdlog::logger> logger;
};

}  // namespace srlivo
}  // namespace asuka
