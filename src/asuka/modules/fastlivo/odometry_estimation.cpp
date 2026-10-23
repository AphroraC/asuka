#include <asuka/modules/fastlivo/odometry_estimation.hpp>

#include <vikit/equidistant_camera.h>
#include <vikit/pinhole_camera.h>

#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace fastlivo {

OdometryEstimation::OdometryEstimation() {
  logger = create_module_logger("odometry");

  const Config odometry_config = load_config("config_odometry");
  const Config ros_config = load_config("config_ros");

  const std::string slam_mode_str = odometry_config.param<std::string>("odometry", "slam_mode", "ONLY_LIO");
  if (slam_mode_str == "ONLY_LIO") {
    param.slam_mode = ONLY_LIO;
  } else if (slam_mode_str == "LIVO") {
    param.slam_mode = LIVO;
  } else {
    throw std::runtime_error("fastlivo: unknown slam_mode: " + slam_mode_str + " (only ONLY_LIO / LIVO are supported)");
  }

  param.gyr_cov = odometry_config.param<double>("imu", "gyr_cov", 1.0);
  param.acc_cov = odometry_config.param<double>("imu", "acc_cov", 1.0);
  param.inv_expo_cov = odometry_config.param<double>("vio", "inv_expo_cov", 0.2);
  param.imu_int_frame = odometry_config.param<int>("imu", "imu_int_frame", 3);
  param.gravity_est_en = odometry_config.param<bool>("imu", "gravity_est_en", true);
  param.ba_bg_est_en = odometry_config.param<bool>("imu", "ba_bg_est_en", true);
  param.exposure_estimate_en = odometry_config.param<bool>("vio", "exposure_estimate_en", true);

  param.filter_size_surf = odometry_config.param<double>("mapping", "filter_size_surf", 0.5);

  param.normal_en = odometry_config.param<bool>("vio", "normal_en", true);
  param.inverse_composition_en = odometry_config.param<bool>("vio", "inverse_composition_en", false);
  param.raycast_en = odometry_config.param<bool>("vio", "raycast_en", false);
  param.max_iterations = odometry_config.param<int>("vio", "max_iterations", 5);
  param.img_point_cov = odometry_config.param<double>("vio", "img_point_cov", 100.0);
  param.grid_size = odometry_config.param<int>("vio", "grid_size", 5);
  param.grid_n_height = odometry_config.param<int>("vio", "grid_n_height", 17);
  param.patch_size = odometry_config.param<int>("vio", "patch_size", 8);
  param.patch_pyrimid_level = odometry_config.param<int>("vio", "patch_pyrimid_level", 3);
  param.outlier_threshold = odometry_config.param<double>("vio", "outlier_threshold", 1000.0);

  param.img_time_offset = odometry_config.param<double>("time_offset", "img_time_offset", 0.0);
  param.exposure_time_init = odometry_config.param<double>("time_offset", "exposure_time_init", 0.0);

  param.blind_rgb_points = odometry_config.param<double>("publish", "blind_rgb_points", 0.01);
  param.dense_map_en = odometry_config.param<bool>("publish", "dense_map_en", false);
  param.hilti_en = odometry_config.param<bool>("preprocess", "hilti_en", false);

  param.save_map_en = ros_config.param<bool>("ros", "enable_map_saving", false);

  param.imu_buffer_capacity = odometry_config.param<int>("odometry", "imu_buffer_capacity", 1000);
  param.lidar_buffer_capacity = odometry_config.param<int>("odometry", "lidar_buffer_capacity", 50);
  param.image_buffer_capacity = odometry_config.param<int>("odometry", "image_buffer_capacity", 50);
  param.measures_capacity = odometry_config.param<int>("odometry", "measures_capacity", 50);

  lidar_buffer = boost::circular_buffer<PointCloudT::Ptr>(param.lidar_buffer_capacity);
  lidar_time_buffer = boost::circular_buffer<double>(param.lidar_buffer_capacity);
  imu_buffer = boost::circular_buffer<ImuData::ConstPtr>(param.imu_buffer_capacity);
  img_buffer = boost::circular_buffer<ImageData::ConstPtr>(param.image_buffer_capacity);
  img_time_buffer = boost::circular_buffer<double>(param.image_buffer_capacity);
  lidar_measures.measures = boost::circular_buffer<MeasureGroup>(param.measures_capacity);

  down_size_filter_surf.setLeafSize(param.filter_size_surf, param.filter_size_surf, param.filter_size_surf);

  // --- extrinsics: lidar -> IMU (the "sensor" block of config_fastlivo) ---
  if (!odometry_config.has("sensor")) {
    throw std::runtime_error("config_odometry: no sensor block");
  }
  const std::vector<double> extrin_t = odometry_config.param<std::vector<double>>("sensor", "extrinsic_T", {0.0, 0.0, 0.0});
  const std::vector<double> extrin_r =
    odometry_config.param<std::vector<double>>("sensor", "extrinsic_R", {1, 0, 0, 0, 1, 0, 0, 0, 1});
  extT << VEC_FROM_ARRAY(extrin_t);
  extR << MAT_FROM_ARRAY(extrin_r);


  // --- voxel map manager (config built in ctor instead of loadVoxelConfig) ---
  VoxelMapConfig voxel_config;
  voxel_config.max_voxel_size_ = odometry_config.param<double>("lio", "voxel_size", 0.5);
  voxel_config.max_layer_ = odometry_config.param<int>("lio", "max_layer", 1);
  voxel_config.planner_threshold_ = odometry_config.param<double>("lio", "min_eigen_value", 0.01);
  voxel_config.sigma_num_ = odometry_config.param<double>("lio", "sigma_num", 3);
  voxel_config.beam_err_ = odometry_config.param<double>("lio", "beam_err", 0.02);
  voxel_config.dept_err_ = odometry_config.param<double>("lio", "dept_err", 0.05);
  voxel_config.layer_init_num_ = odometry_config.param<std::vector<int>>("lio", "layer_init_num", {5, 5, 5, 5, 5});
  voxel_config.max_points_num_ = odometry_config.param<int>("lio", "max_points_num", 50);
  voxel_config.max_iterations_ = odometry_config.param<int>("lio", "max_iterations", 5);
  voxel_config.is_pub_plane_map_ = false;
  voxel_config.map_sliding_en = odometry_config.param<bool>("lio", "map_sliding_en", false);
  voxel_config.half_map_size = odometry_config.param<int>("lio", "half_map_size", 100);
  voxel_config.sliding_thresh = odometry_config.param<double>("lio", "sliding_thresh", 8);

  voxelmap_manager = std::make_shared<VoxelMapManager>(voxel_config, voxel_map);
  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrin_t);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrin_r);

  cloud_preprocess_impl = std::make_shared<CloudPreprocess>();
  p_imu = std::make_shared<ImuPreprocess>();

  // --- VIO manager (LIVO mode only; the camera block of config_fastlivo gains
  // its first real reader here) ---
  if (param.slam_mode == LIVO) {
    vio_manager = std::make_shared<VIOManager>();
    camera.reset(make_camera(odometry_config));
    if (!camera) throw std::runtime_error("fastlivo: camera model not correctly specified (config_fastlivo camera block)");

    // lidar -> camera extrinsic: compose the camera pose in the IMU frame
    // (config_fastlivo camera block, T_imu_camera) with the lidar -> IMU
    // extrinsic read above: R_cl = R_ic^T * R_il, t_cl = R_ic^T * (t_il - t_ic).
    const Eigen::Isometry3d t_imu_camera =
      odometry_config.param_nested<Eigen::Isometry3d>({std::string("camera")}, "T_imu_camera", Eigen::Isometry3d::Identity());
    const Eigen::Matrix3d r_cl = t_imu_camera.linear().transpose() * extR;
    const Eigen::Vector3d t_cl = t_imu_camera.linear().transpose() * (extT - t_imu_camera.translation());
    // Eigen Eigen::Matrix3d stores column-major while setLidarToCameraExtrinsic()
    // reads the std::vector row-major (MAT_FROM_ARRAY); flatten row-major to keep
    // the rotation identical to the upstream yaml.
    const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> r_cl_row(r_cl);
    cameraextrinR = std::vector<double>(r_cl_row.data(), r_cl_row.data() + 9);
    cameraextrinT = std::vector<double>(t_cl.data(), t_cl.data() + 3);

    vio_manager->grid_size = param.grid_size;
    vio_manager->patch_size = param.patch_size;
    vio_manager->outlier_threshold = param.outlier_threshold;
    vio_manager->setImuToLidarExtrinsic(extT, extR);
    vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
    vio_manager->state = &state;
    vio_manager->state_propagat = &state_propagat;
    vio_manager->max_iterations = param.max_iterations;
    vio_manager->img_point_cov = param.img_point_cov;
    vio_manager->normal_en = param.normal_en;
    vio_manager->inverse_composition_en = param.inverse_composition_en;
    vio_manager->raycast_en = param.raycast_en;
    vio_manager->grid_n_width = 0;  // recomputed inside initializeVIO
    vio_manager->grid_n_height = param.grid_n_height;
    vio_manager->patch_pyrimid_level = param.patch_pyrimid_level;
    vio_manager->exposure_estimate_en = param.exposure_estimate_en;
    vio_manager->cam = camera.get();
    vio_manager->initializeVIO();
  }

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(Eigen::Vector3d(param.gyr_cov, param.gyr_cov, param.gyr_cov));
  p_imu->set_acc_cov_scale(Eigen::Vector3d(param.acc_cov, param.acc_cov, param.acc_cov));
  p_imu->set_inv_expo_cov(param.inv_expo_cov);
  p_imu->set_gyr_bias_cov(Eigen::Vector3d(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(Eigen::Vector3d(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(param.imu_int_frame);

  if (!param.gravity_est_en) p_imu->disable_gravity_est();
  if (!param.ba_bg_est_en) p_imu->disable_bias_est();
  if (!param.exposure_estimate_en) p_imu->disable_exposure_est();

  logger->info("fastlivo initialized: slam_mode={} filter_size_surf={}", slam_mode_str, param.filter_size_surf);
}

OdometryEstimation::~OdometryEstimation() { stop(); }

vk::AbstractCamera* OdometryEstimation::make_camera(const Config& config) {
  std::vector<std::string> nest{"camera"};
  if (!config.has("camera")) {
    logger->error("fastlivo: LIVO mode requires the config_fastlivo camera block");
    return nullptr;
  }
  const std::vector<int> image_size = config.param_nested<std::vector<int>>(nest, "image_size", {1280, 720});
  const std::vector<double> intrinsics = config.param_nested<std::vector<double>>(nest, "intrinsics", {1000.0, 1000.0, 640.0, 360.0});
  const double scale = config.param_nested<double>(nest, "scale", 1.0);
  const std::string model = config.param_nested<std::string>(nest, "distortion_model", "plumb_bob");
  const std::vector<double> coeffs = config.param_nested<std::vector<double>>(nest, "distortion_coeffs", {0.0, 0.0, 0.0, 0.0});

  if (intrinsics.size() < 4) {
    logger->error("fastlivo: camera intrinsics must be [fx, fy, cx, cy]");
    return nullptr;
  }
  auto coeff = [&](size_t i) { return i < coeffs.size() ? coeffs[i] : 0.0; };

  if (model == "plumb_bob" || model == "pinhole" || model == "Pinhole") {
    return new vk::PinholeCamera(image_size[0], image_size[1], scale, intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3], coeff(0),
                                 coeff(1), coeff(2), coeff(3));
  } else if (model == "equidistant" || model == "EquidistantCamera") {
    return new vk::EquidistantCamera(image_size[0], image_size[1], scale, intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3], coeff(0),
                                     coeff(1), coeff(2), coeff(3));
  }
  logger->error("fastlivo: unsupported distortion_model: {}", model);
  return nullptr;
}

// --- input sides of the upstream callbacks ---

void OdometryEstimation::insert_imu(const ImuData::ConstPtr& imu) {
  // Upstream imu_cbk: IMU before the first lidar frame is dropped, and the
  // IMU/lidar sync warning is emitted without any auto-correction (the
  // ros_driver_bug_fix path is not ported).
  if (last_timestamp_lidar < 0.0) return;
  const double timestamp = imu->stamp;

  if (fabs(last_timestamp_lidar - timestamp) > 0.5) {
    logger->warn("IMU and LiDAR not synced! delta time: {:.6f}", last_timestamp_lidar - timestamp);
  }

  std::lock_guard<std::mutex> lock(buffer_mutex);
  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu) {
    logger->error("imu loop back, offset: {:.6f}", last_timestamp_imu - timestamp);
    return;
  }
  last_timestamp_imu = timestamp;
  imu_buffer.push_back(imu);
}

void OdometryEstimation::insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  // Upstream standard_pcl_cbk / livox_pcl_cbk: on loop back the raw lidar
  // buffer is cleared and the looped-back frame is accepted.
  if (stamp < last_timestamp_lidar) {
    logger->warn("lidar loop back, clear buffer");
    lidar_buffer.clear();
    lidar_time_buffer.clear();
  }
  if (!cloud || cloud->empty()) {
    logger->warn("Received an empty point cloud");
    return;
  }
  // Executor hands over a ConstPtr; the sync state machine mutates the
  // buffered scan through the pcl_proc_cur alias, so own a mutable copy.
  PointCloudT::Ptr mutable_cloud(new PointCloudT(*cloud));
  lidar_buffer.push_back(mutable_cloud);
  lidar_time_buffer.push_back(stamp);
  last_timestamp_lidar = stamp;
}

void OdometryEstimation::insert_image(const ImageData::ConstPtr& image) {
  if (!image) return;
  // Upstream img_cbk algorithm-level filter chain, in the original order.
  if (param.hilti_en) {
    if (++hilti_frame_counter % 4 != 0) return;
  }
  const double msg_header_time = image->stamp + param.img_time_offset;
  if (fabs(msg_header_time - last_timestamp_img) < 0.001) return;
  if (last_timestamp_lidar < 0) return;
  if (msg_header_time < last_timestamp_img) {
    logger->error("image loop back.");
    return;
  }
  if (msg_header_time - last_timestamp_img < 0.02) {
    logger->warn("Image need Jumps: {:.6f}", msg_header_time);
    return;
  }
  std::lock_guard<std::mutex> lock(buffer_mutex);
  img_buffer.push_back(image);
  img_time_buffer.push_back(msg_header_time);
  last_timestamp_img = msg_header_time;
}

int OdometryEstimation::workload() const {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  // Only the lidar depth feeds the rosbag player's back-pressure threshold:
  // IMU/image events arrive far faster than sync_packages drains them, and
  // counting them would stall the player on the no-progress timeout (same
  // convention as the other frontends).
  return static_cast<int>(lidar_buffer.size());
}

void OdometryEstimation::stop() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  imu_buffer.clear();
  lidar_buffer.clear();
  lidar_time_buffer.clear();
  img_buffer.clear();
  img_time_buffer.clear();
  lidar_pushed = false;
}

PointCloudT::Ptr OdometryEstimation::save_map() {
  if (!param.save_map_en) return nullptr;
  std::lock_guard<std::mutex> lock(buffer_mutex);
  return pcl_wait_save;
}

bool OdometryEstimation::process_once() {
  // One upstream run()-loop iteration: sync, then estimate. The executor
  // calls this after every delivered event, replacing the 5000 Hz polling.
  std::lock_guard<std::mutex> lock(buffer_mutex);
  if (!sync_packages(lidar_measures)) return false;
  handle_first_frame();
  process_imu();
  state_estimation_and_mapping();
  return true;
}

// --- sync_packages: verbatim port of the LIVMapper state machine ---

bool OdometryEstimation::sync_packages(LidarMeasureGroup& meas) {
  if (lidar_buffer.empty()) return false;
  if (img_buffer.empty() && param.slam_mode == LIVO) return false;
  if (imu_buffer.empty()) return false;

  switch (param.slam_mode) {
    case ONLY_LIO: {
      if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lidar_time_buffer.front();
      if (!lidar_pushed) {
        // If not push the lidar into measurement data buffer
        meas.lidar = lidar_buffer.front();  // push the first lidar topic
        if (meas.lidar->points.size() <= 1) return false;

        meas.lidar_frame_beg_time = lidar_time_buffer.front();                                            // generate lidar_frame_beg_time
        meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000);  // calc lidar scan end time
        meas.pcl_proc_cur = meas.lidar;
        lidar_pushed = true;  // flag
      }

      if (last_timestamp_imu < meas.lidar_frame_end_time) {
        // waiting imu message needs to be larger than lidar_frame_end_time,
        // make sure complete propagate.
        return false;
      }

      struct MeasureGroup m;  // standard method to keep imu message.

      m.imu.clear();
      m.lio_time = meas.lidar_frame_end_time;
      while (!imu_buffer.empty()) {
        if (imu_buffer.front()->stamp > meas.lidar_frame_end_time) break;
        m.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
      }
      lidar_buffer.pop_front();
      lidar_time_buffer.pop_front();

      meas.lio_vio_flg = LIO;  // process lidar topic, so timestamp should be lidar scan end.
      meas.measures.push_back(m);
      lidar_pushed = false;  // sync one whole lidar scan.
      return true;
    }

    case LIVO: {
      /*** For LIVO mode, the time of LIO update is set to be the same as VIO, LIO
       * first than VIO imediatly ***/
      EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
      switch (last_lio_vio_flg) {
        case WAIT:
        case VIO: {
          double img_capture_time = img_time_buffer.front() + param.exposure_time_init;
          /*** has img topic, but img topic timestamp larger than lidar end time,
           * process lidar topic. After LIO update, the meas.lidar_frame_end_time
           * will be refresh. ***/
          if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lidar_time_buffer.front();

          double lid_newest_time = lidar_time_buffer.back() + lidar_buffer.back()->points.back().curvature / double(1000);
          double imu_newest_time = imu_buffer.back()->stamp;

          if (img_capture_time < meas.last_lio_update_time + 0.00001) {
            img_buffer.pop_front();
            img_time_buffer.pop_front();
            logger->warn("[ Data Cut ] Throw one image frame!");
            return false;
          }

          if (img_capture_time > lid_newest_time || img_capture_time > imu_newest_time) {
            // Upstream silently waits here: with images enabled the radar
            // frames can only be consumed driven by an image timestamp.
            if (++livo_wait_count % 500 == 1) {
              logger->warn("LIVO sync waiting for data coverage (img_capture_time {:.3f} vs lid {:.3f} imu {:.3f})", img_capture_time,
                           lid_newest_time, imu_newest_time);
            }
            return false;
          }

          struct MeasureGroup m;

          m.imu.clear();
          m.lio_time = img_capture_time;
          while (!imu_buffer.empty()) {
            if (imu_buffer.front()->stamp > m.lio_time) break;

            if (imu_buffer.front()->stamp > meas.last_lio_update_time) m.imu.push_back(imu_buffer.front());

            imu_buffer.pop_front();
          }

          *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
          PointCloudT().swap(*meas.pcl_proc_next);

          int lid_frame_num = lidar_buffer.size();
          int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
          meas.pcl_proc_cur->reserve(max_size);
          meas.pcl_proc_next->reserve(max_size);

          // KeyFrame::stamp contract: remember the header time of the first
          // scan merged into this window. When no scan merges (image older
          // than every buffered scan), keep the previous window's value.
          if (!lidar_buffer.empty() && lidar_time_buffer.front() <= img_capture_time) {
            meas.lidar_frame_beg_time = lidar_time_buffer.front();
          }
          while (!lidar_buffer.empty()) {
            if (lidar_time_buffer.front() > img_capture_time) break;
            auto pcl(*lidar_buffer.front());
            double frame_header_time(lidar_time_buffer.front());
            float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

            for (int i = 0; i < static_cast<int>(pcl.size()); i++) {
              auto pt = pcl[i];
              if (pcl[i].curvature < max_offs_time_ms) {
                pt.curvature += (frame_header_time - meas.last_lio_update_time) * 1000.0f;
                meas.pcl_proc_cur->points.push_back(pt);
              } else {
                pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
                meas.pcl_proc_next->points.push_back(pt);
              }
            }
            lidar_buffer.pop_front();
            lidar_time_buffer.pop_front();
          }

          meas.measures.push_back(m);
          meas.lio_vio_flg = LIO;
          return true;
        }

        case LIO: {
          double img_capture_time = img_time_buffer.front() + param.exposure_time_init;
          meas.lio_vio_flg = VIO;
          meas.measures.clear();
          double imu_time = imu_buffer.front()->stamp;
          (void)imu_time;

          struct MeasureGroup m;
          m.vio_time = img_capture_time;
          m.lio_time = meas.last_lio_update_time;
          m.img = *img_buffer.front()->image;
          img_buffer.pop_front();
          img_time_buffer.pop_front();
          meas.measures.push_back(m);
          lidar_pushed = false;  // after VIO update, the lidar_frame_end_time will be refresh.
          return true;
        }

        default: {
          return false;
        }
      }
    }

    default: {
      logger->error("!! WRONG SLAM TYPE !!");
      return false;
    }
  }
}

// --- estimation pipeline (LIVMapper core) ---

void OdometryEstimation::handle_first_frame() {
  if (!is_first_frame) {
    first_lidar_time = lidar_measures.last_lio_update_time;
    p_imu->first_lidar_time = first_lidar_time;  // Only for IMU data log
    is_first_frame = true;
    logger->info("FIRST LIDAR FRAME!");
  }
}

void OdometryEstimation::process_imu() {
  p_imu->Process2(lidar_measures, state, feats_undistort);

  state_propagat = state;
  voxelmap_manager->state_ = state;
  voxelmap_manager->feats_undistort_ = feats_undistort;
}

void OdometryEstimation::state_estimation_and_mapping() {
  switch (lidar_measures.lio_vio_flg) {
    case VIO:
      handle_vio();
      break;
    case LIO:
    case LO:
      handle_lio();
      break;
    default:
      break;
  }
}

void OdometryEstimation::handle_vio() {
  euler_cur = RotMtoEuler(state.rot_end);

  if (pcl_w_wait_pub->empty()) {
    logger->warn("[ VIO ] No point!!!");
    return;
  }

  logger->info("[ VIO ] Raw feature num: {}", pcl_w_wait_pub->points.size());

  try {
    vio_manager->processFrame(lidar_measures.measures.back().img, pv_list, voxelmap_manager->voxel_map_,
                              lidar_measures.last_lio_update_time - first_lidar_time);
  } catch (const std::exception& e) {
    // A frame that disagrees with the camera model (size mismatch etc.) must
    // degrade to a skipped update, not take the odometry worker down; the
    // critical log makes a persistent calibration mismatch loud.
    logger->critical("[ VIO ] processFrame exception, frame skipped: {}", e.what());
    return;
  }

  // One KeyFrame per VIO EKF update. The LIO frame's cloud was already
  // emitted and shared (immutable), so the VIO frame carries a fresh colored
  // copy -- upstream colors only the VIO-branch output as well.
  PointCloudT::Ptr colored(new PointCloudT(*current_cloud_imu));
  colorize_cloud(*colored);
    // KeyFrame::stamp contract: sweep start (see callbacks.hpp on_new_frame).
  emit_key_frame(lidar_measures.lidar_frame_beg_time, colored);

  if (param.save_map_en && param.slam_mode == LIVO) accumulate_colored_map();
}

void OdometryEstimation::handle_lio() {
  euler_cur = RotMtoEuler(state.rot_end);

  if (feats_undistort->empty()) {
    logger->warn("[ LIO ]: No point!!!");
    return;
  }

  down_size_filter_surf.setInputCloud(feats_undistort);
  down_size_filter_surf.filter(*feats_down_body);

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transform_lidar(state.rot_end, state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;

  if (!lidar_map_inited) {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  voxelmap_manager->StateEstimation(state_propagat);
  state = voxelmap_manager->state_;
  pv_list = voxelmap_manager->pv_list_;

  // Refine the world points/variances with the converged state, then fold
  // the scan into the voxel map.
  PointCloudT::Ptr world_lidar(new PointCloudT());
  transform_lidar(state.rot_end, state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++) {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    Eigen::Matrix3d point_crossmat = voxelmap_manager->cross_mat_list_[i];
    Eigen::Matrix3d var = voxelmap_manager->body_cov_list_[i];
    var = (state.rot_end * extR) * var * (state.rot_end * extR).transpose() + (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
          (-point_crossmat).transpose() + state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
  pv_list = voxelmap_manager->pv_list_;

  if (voxelmap_manager->config_setting_.map_sliding_en) {
    voxelmap_manager->mapSliding();
  }

  // World-frame cloud; upstream laserCloudFullRes = dense ? feats_undistort
  // : feats_down_body.
  const PointCloudT::Ptr& laser_cloud_full_res = param.dense_map_en ? feats_undistort : feats_down_body;
  PointCloudT::Ptr laser_cloud_world(new PointCloudT(laser_cloud_full_res->points.size(), 1));
  for (size_t i = 0; i < laser_cloud_full_res->points.size(); i++) {
    point_body_to_world(laser_cloud_full_res->points[i], laser_cloud_world->points[i]);
  }
  *pcl_w_wait_pub = *laser_cloud_world;

  // One KeyFrame per LIO EKF update with a fresh IMU-frame cloud; the same
  // shared buffer is handed to the VIO frames of this scan.
  current_cloud_imu.reset(new PointCloudT(*feats_undistort));
  // Upstream publishes nothing at LIO updates in LIVO mode -- the colored
  // cloud only comes with the VIO update; ONLY_LIO maps to the intensity
  // stream. The LIO KeyFrame still anchors imu_prediction (cloud unused).
    // KeyFrame::stamp contract: sweep start (see callbacks.hpp on_new_frame).
  emit_key_frame(lidar_measures.lidar_frame_beg_time,
                 param.slam_mode == LIVO ? PointCloudT::Ptr() : current_cloud_imu);

  if (param.save_map_en && param.slam_mode != LIVO) {
    *pcl_wait_save += *pcl_w_wait_pub;
  }
}

void OdometryEstimation::colorize_cloud(PointCloudT& cloud) {
  if (!vio_manager || vio_manager->img_rgb.empty()) return;
  const cv::Mat img_rgb = vio_manager->img_rgb;
  for (size_t i = 0; i < cloud.points.size(); i++) {
    PointT& point = cloud.points[i];
    const Eigen::Vector3d p_body(point.x, point.y, point.z);
    const Eigen::Vector3d p_w(state.rot_end * (extR * p_body + extT) + state.pos_end);
    const Eigen::Vector3d pf(vio_manager->new_frame_->w2f(p_w));
    if (pf[2] < 0) continue;
    const Eigen::Vector2d pc(vio_manager->new_frame_->w2c(p_w));
    if (!vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) continue;
    const Eigen::Vector3f pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
    point.b = pixel[0];
    point.g = pixel[1];
    point.r = pixel[2];
  }
}

void OdometryEstimation::emit_key_frame(double stamp, const PointCloudT::Ptr& cloud_imu) {
  auto frame = std::make_shared<asuka::KeyFrame>();
  frame->id = frame_counter++;
  frame->stamp = stamp;
  frame->frame_id = FrameId::IMU;
  frame->T_world_imu = Eigen::Isometry3d::Identity();
  frame->T_world_imu.linear() = state.rot_end;
  frame->T_world_imu.translation() = state.pos_end;
  frame->v_world_imu = state.vel_end;
  frame->imu_bias.head<3>() = state.bias_g;
  frame->imu_bias.tail<3>() = state.bias_a;
  frame->cloud_imu = cloud_imu;

  Callbacks::on_new_frame(frame);
}

// Upstream publish_frame_world coloring branch (pub_scan_num = 1): project
// the scan's world points into the current camera frame and pick the BGR
// pixel by bilinear interpolation.
void OdometryEstimation::accumulate_colored_map() {
  if (pcl_w_wait_pub->empty()) return;
  cv::Mat img_rgb = vio_manager->img_rgb;
  for (size_t i = 0; i < pcl_w_wait_pub->points.size(); i++) {
    PointT point_rgb;
    point_rgb.x = pcl_w_wait_pub->points[i].x;
    point_rgb.y = pcl_w_wait_pub->points[i].y;
    point_rgb.z = pcl_w_wait_pub->points[i].z;
    point_rgb.intensity = pcl_w_wait_pub->points[i].intensity;

    Eigen::Vector3d p_w(pcl_w_wait_pub->points[i].x, pcl_w_wait_pub->points[i].y, pcl_w_wait_pub->points[i].z);
    Eigen::Vector3d pf(vio_manager->new_frame_->w2f(p_w));
    if (pf[2] < 0) continue;
    Eigen::Vector2d pc(vio_manager->new_frame_->w2c(p_w));

    if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) {
      Eigen::Vector3f pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
      point_rgb.r = pixel[2];
      point_rgb.g = pixel[1];
      point_rgb.b = pixel[0];
      if (pf.norm() > param.blind_rgb_points) pcl_wait_save->push_back(point_rgb);
    }
  }
}

void OdometryEstimation::transform_lidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudT::Ptr& input_cloud,
                                         PointCloudT::Ptr& trans_cloud) {
  PointCloudT().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++) {
    const PointT& p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointT pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void OdometryEstimation::point_body_to_world(const PointT& pi, PointT& po) {
  Eigen::Vector3d p_body(pi.x, pi.y, pi.z);
  Eigen::Vector3d p_global(state.rot_end * (extR * p_body + extT) + state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

}  // namespace fastlivo
}  // namespace asuka
