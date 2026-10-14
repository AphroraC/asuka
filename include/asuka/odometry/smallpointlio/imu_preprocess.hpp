#pragma once

#include <deque>
#include <memory>

#include <asuka/core/imu_preprocess.hpp>
#include <asuka/core/types.hpp>
#include <asuka/utility/logging.hpp>

namespace asuka {
namespace smallpointlio {

class ImuPreprocess : public asuka::ImuPreprocess {
public:
  ImuPreprocess();
  ~ImuPreprocess() override = default;

  void insert_imu(const ImuData::ConstPtr& imu) override;

  std::deque<ImuData::ConstPtr> imu_deque;

private:
  double last_timestamp_imu{-1.0};
  std::shared_ptr<spdlog::logger> logger{nullptr};
};

}  // namespace smallpointlio
}  // namespace asuka
