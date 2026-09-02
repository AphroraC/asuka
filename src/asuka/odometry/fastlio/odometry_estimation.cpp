#include <asuka/odometry/fastlio/odometry_estimation.hpp>

#include <algorithm>

#include <asuka/utility/config_impl.hpp>

namespace asuka {
namespace fastlio {

OdometryEstimation::OdometryEstimation() {
  logger = create_module_logger("odometry");
  imu_preprocess = std::make_shared<ImuPreprocess>();
  cloud_preprocess_impl = std::make_shared<CloudPreprocess>();

  const Config odometry_config(GlobalConfig::get_config_path("config_odometry"));
  max_iterations = odometry_config.param<int>("odometry", "max_iteration", 4);
  laser_point_cov = odometry_config.param<double>("odometry", "laser_point_cov", 0.001);
  const Config ros_config(GlobalConfig::get_config_path("config_ros"));
  save_map_en = ros_config.param<bool>("ros", "enable_map_saving", false);
  fov_degree = odometry_config.param<double>("mapping", "fov_degree", 180.0);

  filter_size_surf = odometry_config.param<double>("filter", "filter_size_surf", 0.5);
  filter_size_map = odometry_config.param<double>("filter", "filter_size_map", 0.5);
  cube_len = odometry_config.param<double>("filter", "cube_side_length", 200.0);

  det_range = odometry_config.param<double>("preprocess", "max_distance", 100.0);

  lidar_buffer.set_capacity(odometry_config.param<int>("odometry", "lidar_buffer_capacity", 50));
  time_buffer.set_capacity(odometry_config.param<int>("odometry", "lidar_buffer_capacity", 50));
  imu_buffer.set_capacity(odometry_config.param<int>("odometry", "imu_buffer_capacity", 1000));

  downsize_filter_surf.setLeafSize(filter_size_surf, filter_size_surf, filter_size_surf);
  ikdtree.set_downsample_param(filter_size_map);

  map_cloud.reset(new PointCloudT());
  map_cloud->width = map_cloud->height = 0;
  map_cloud->is_dense = false;

  std::fill(epsi, epsi + 23, 0.001);
  kf.init_dyn_share(
    ikfom::get_f,
    ikfom::df_dx,
    ikfom::df_dw,
    &OdometryEstimation::h_share_model_trampoline,
    max_iterations,
    epsi);

  logger->debug("fastlio::OdometryEstimation initialized");
}

OdometryEstimation::~OdometryEstimation() {
  stop();
}

void OdometryEstimation::stop() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  lidar_buffer.clear();
  time_buffer.clear();
  imu_buffer.clear();
  lidar_pushed = false;
}

void OdometryEstimation::clear_buffers() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  lidar_buffer.clear();
  time_buffer.clear();
  imu_buffer.clear();
  lidar_pushed = false;
  last_timestamp_imu = -1.0;
}

PointCloudT::Ptr OdometryEstimation::save_map() {
  std::lock_guard<std::mutex> lock(buffer_mutex);
  return map_cloud;
}

void OdometryEstimation::insert_imu(const ImuData::ConstPtr& imu) {
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    last_timestamp_imu = imu->stamp;
    imu_buffer.push_back(imu);
  }
  Callbacks::on_insert_imu(imu);
}

void OdometryEstimation::insert_frame(double stamp, const PointCloudT::ConstPtr& cloud) {
  {
    std::lock_guard<std::mutex> lock(buffer_mutex);
    lidar_buffer.push_back(cloud);
    time_buffer.push_back(stamp);
  }
  Callbacks::on_insert_frame(stamp, cloud);
}

int OdometryEstimation::workload() const {
  return static_cast<int>(lidar_buffer.size());
}

bool OdometryEstimation::process_once() {
  active = this;
  ImuMeasureGroup measures;
  if (!sync_packages(measures)) return false;
  process_scan(measures);
  return true;
}

bool OdometryEstimation::sync_packages(ImuMeasureGroup& measures) {
  std::lock_guard<std::mutex> lock(buffer_mutex);

  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  if (!lidar_pushed) {
    pending_lidar = lidar_buffer.front();
    pending_lidar_beg_time = time_buffer.front();

    if (pending_lidar->points.size() <= 1) {
      lidar_end_time = pending_lidar_beg_time + lidar_mean_scantime;
      logger->warn("Too few input point cloud!");
    } else if (pending_lidar->points.back().curvature / 1000.0 < 0.5 * lidar_mean_scantime) {
      lidar_end_time = pending_lidar_beg_time + lidar_mean_scantime;
    } else {
      scan_num++;
      lidar_end_time = pending_lidar_beg_time + pending_lidar->points.back().curvature / 1000.0;
      lidar_mean_scantime += (pending_lidar->points.back().curvature / 1000.0 - lidar_mean_scantime) / scan_num;
    }
    pending_lidar_end_time = lidar_end_time;
    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  double imu_time = imu_buffer.front()->stamp;
  measures.imu.clear();
  while (!imu_buffer.empty() && imu_time < lidar_end_time) {
    imu_time = imu_buffer.front()->stamp;
    if (imu_time > lidar_end_time) break;
    measures.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  measures.lidar = pending_lidar;
  measures.lidar_beg_time = pending_lidar_beg_time;
  measures.lidar_end_time = pending_lidar_end_time;

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

void OdometryEstimation::process_scan(ImuMeasureGroup& measures) {
  if (first_scan_flag) {
    first_lidar_time = measures.lidar_beg_time;
    imu_preprocess->initialize(first_lidar_time);
    first_scan_flag = false;
    return;
  }

  feats_undistort->clear();
  imu_preprocess->process(measures, kf, feats_undistort);

  state_point = kf.get_x();
  pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

  if (!feats_undistort || feats_undistort->empty()) {
    logger->warn("No points, skip this scan!");
    return;
  }

  ekf_inited_flag = (measures.lidar_beg_time - first_lidar_time) >= init_time;

  lasermap_fov_segment();

  feats_down_body->clear();
  downsize_filter_surf.setInputCloud(feats_undistort);
  downsize_filter_surf.filter(*feats_down_body);
  feats_down_size = static_cast<int>(feats_down_body->points.size());

  if (ikdtree.Root_Node == nullptr) {
    if (feats_down_size > 5) {
      feats_down_world->resize(feats_down_size);
      for (int i = 0; i < feats_down_size; ++i) {
        point_body_to_world(feats_down_body->points[i], feats_down_world->points[i]);
      }
      ikdtree.Build(feats_down_world->points);
    }
    return;
  }

  if (feats_down_size < 5) {
    logger->warn("No point, skip this scan!");
    return;
  }

  normvec->resize(feats_down_size);
  feats_down_world->resize(feats_down_size);
  nearest_points.resize(feats_down_size);
  point_selected_surf.assign(feats_down_size, true);
  res_last.assign(feats_down_size, -1000.0f);

  double solve_H_time = 0.0;
  kf.update_iterated_dyn_share_modified(laser_point_cov, solve_H_time);

  state_point = kf.get_x();
  euler_cur = ikfom::SO3ToEuler(state_point.rot);
  pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

  map_incremental();

  Eigen::Isometry3d T_world_imu(state_point.rot.toRotationMatrix());
  T_world_imu.pretranslate(state_point.pos);

  const int undistort_size = static_cast<int>(feats_undistort->size());
  cloud_world->resize(undistort_size);
  cloud_body->resize(undistort_size);

#ifdef FASTLIO_ENABLE_OMP
#pragma omp parallel for num_threads(FASTLIO_NUM_THREADS) schedule(static)
#endif
  for (int i = 0; i < undistort_size; ++i) {
    point_body_to_world(feats_undistort->points[i], cloud_world->points[i]);
    point_body_lidar_to_imu(feats_undistort->points[i], cloud_body->points[i]);
  }

  auto frame = std::make_shared<KeyFrame>();
  frame->id = frame_counter++;
  frame->stamp = measures.lidar_end_time;
  frame->frame_id = FrameId::IMU;
  frame->T_world_imu = T_world_imu;
  frame->v_world_imu = state_point.vel;
  frame->imu_bias.head<3>() = state_point.bg;
  frame->imu_bias.tail<3>() = state_point.ba;
  // cloud_body is a reused member buffer: give the frame its own copy so
  // observers may hold it for as long as they like.
  frame->cloud_imu = PointCloudT::Ptr(new PointCloudT(*cloud_body));

  Callbacks::on_new_frame(frame);

  if (save_map_en) {
    *map_cloud += *cloud_world;
  }

  logger->debug(
    "[lio] scan stamp={:.3f} feats_down={} eff={} map_size={} scan_dt={:.3f}s",
    measures.lidar_beg_time,
    feats_down_size,
    effct_feat_num,
    ikdtree.size(),
    measures.lidar_end_time - measures.lidar_beg_time);
}

void OdometryEstimation::lasermap_fov_segment() {
  cub_needrm.clear();
  PointT x_axis_body;
  x_axis_body.x = lidar_sp_len;
  x_axis_body.y = 0.0f;
  x_axis_body.z = 0.0f;
  PointT x_axis_world;
  point_body_to_world(x_axis_body, x_axis_world);

  if (!localmap_initialized) {
    for (int i = 0; i < 3; ++i) {
      local_map_points.vertex_min[i] = pos_lid(i) - cube_len / 2.0;
      local_map_points.vertex_max[i] = pos_lid(i) + cube_len / 2.0;
    }
    localmap_initialized = true;
    return;
  }

  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; ++i) {
    dist_to_map_edge[i][0] = std::fabs(pos_lid(i) - local_map_points.vertex_min[i]);
    dist_to_map_edge[i][1] = std::fabs(pos_lid(i) - local_map_points.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= mov_threshold * det_range || dist_to_map_edge[i][1] <= mov_threshold * det_range) {
      need_move = true;
    }
  }
  if (!need_move) return;

  BoxPointType new_localmap = local_map_points;
  const float mov_dist =
    std::max((cube_len - 2.0 * mov_threshold * det_range) * 0.5 * 0.9, det_range * (mov_threshold - 1));
  for (int i = 0; i < 3; ++i) {
    BoxPointType tmp_boxpoints = local_map_points;
    if (dist_to_map_edge[i][0] <= mov_threshold * det_range) {
      new_localmap.vertex_max[i] -= mov_dist;
      new_localmap.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = local_map_points.vertex_max[i] - mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= mov_threshold * det_range) {
      new_localmap.vertex_max[i] += mov_dist;
      new_localmap.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = local_map_points.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
  }
  local_map_points = new_localmap;

  if (!cub_needrm.empty()) {
    PointVectorT removed_points;
    ikdtree.acquire_removed_points(removed_points);
    ikdtree.Delete_Point_Boxes(cub_needrm);
  }
}

void OdometryEstimation::point_body_to_world(const PointT& pi, PointT& po) {
  const Eigen::Vector3d p_body(pi.x, pi.y, pi.z);
  const Eigen::Vector3d p_global =
    state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos;
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

void OdometryEstimation::point_body_lidar_to_imu(const PointT& pi, PointT& po) {
  const Eigen::Vector3d p_body_lidar(pi.x, pi.y, pi.z);
  const Eigen::Vector3d p_body_imu = state_point.offset_R_L_I * p_body_lidar + state_point.offset_T_L_I;
  po.x = p_body_imu(0);
  po.y = p_body_imu(1);
  po.z = p_body_imu(2);
  po.intensity = pi.intensity;
}

void OdometryEstimation::h_share_model_trampoline(ikfom::state_ikfom& s, esekfom::dyn_share_datastruct<double>& data) {
  active->h_share_model(s, data);
}

void OdometryEstimation::h_share_model(ikfom::state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data) {
  laser_cloud_ori->resize(feats_down_size);
  corr_normvect->resize(feats_down_size);
  total_residual = 0.0;

#ifdef FASTLIO_ENABLE_OMP
#pragma omp parallel for num_threads(FASTLIO_NUM_THREADS)
#endif
  for (int i = 0; i < feats_down_size; ++i) {
    PointT& point_body = feats_down_body->points[i];
    PointT& point_world = feats_down_world->points[i];

    const Eigen::Vector3d p_body(point_body.x, point_body.y, point_body.z);
    const Eigen::Vector3d p_global = s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos;
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;

    auto& sq_dis = tls_search_sq_dis();
    sq_dis.assign(num_match_points, 0.0f);
    auto& points_near = nearest_points[i];

    if (ekfom_data.converge) {
      ikdtree.Nearest_Search(point_world, num_match_points, points_near, sq_dis);
      point_selected_surf[i] = points_near.size() < num_match_points ? false
                               : sq_dis[num_match_points - 1] > 5.0f ? false
                                                                     : true;
    }

    if (!point_selected_surf[i]) continue;

    Eigen::Vector4f pabcd;
    point_selected_surf[i] = false;
    if (estimate_plane(pabcd, points_near, 0.1f)) {
      const float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
      const float score = 1.0f - 0.9f * std::fabs(pd2) / std::sqrt(p_body.norm());
      if (score > 0.9f) {
        point_selected_surf[i] = true;
        normvec->points[i].x = pabcd(0);
        normvec->points[i].y = pabcd(1);
        normvec->points[i].z = pabcd(2);
        normvec->points[i].intensity = pd2;
        res_last[i] = std::fabs(pd2);
      }
    }
  }

  effct_feat_num = 0;
  for (int i = 0; i < feats_down_size; ++i) {
    if (point_selected_surf[i]) {
      laser_cloud_ori->points[effct_feat_num] = feats_down_body->points[i];
      corr_normvect->points[effct_feat_num] = normvec->points[i];
      total_residual += res_last[i];
      effct_feat_num++;
    }
  }

  if (effct_feat_num < 1) {
    ekfom_data.valid = false;
    logger->warn("No effective points!");
    return;
  }

  res_mean_last = total_residual / effct_feat_num;

  ekfom_data.h_x.resize(effct_feat_num, 12);
  ekfom_data.h_x.setZero();
  ekfom_data.h.resize(effct_feat_num);

  for (int i = 0; i < effct_feat_num; ++i) {
    const PointT& laser_p = laser_cloud_ori->points[i];
    const Eigen::Vector3d point_this_be(laser_p.x, laser_p.y, laser_p.z);
    Eigen::Matrix3d point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    const Eigen::Vector3d point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    Eigen::Matrix3d point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    const PointT& norm_p = corr_normvect->points[i];
    const Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);

    const Eigen::Vector3d C(s.rot.conjugate() * norm_vec);
    const Eigen::Vector3d A(point_crossmat * C);
    // if (extrinsic_est_en) {
    //   const Eigen::Vector3d B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
    //   ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, A(0), A(1), A(2), B(0), B(1), B(2), C(0),
    //   C(1),
    //     C(2);
    // } else {
    ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, A(0), A(1), A(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // }
    ekfom_data.h(i) = -norm_p.intensity;
  }
}

void OdometryEstimation::map_incremental() {
  PointVectorT point_to_add;
  PointVectorT point_no_need_downsample;
  point_to_add.reserve(feats_down_size);
  point_no_need_downsample.reserve(feats_down_size);

  for (int i = 0; i < feats_down_size; ++i) {
    point_body_to_world(feats_down_body->points[i], feats_down_world->points[i]);
    if (!nearest_points[i].empty() && ekf_inited_flag) {
      const PointVectorT& points_near = nearest_points[i];
      bool need_add = true;
      PointT mid_point;
      mid_point.x =
        std::floor(feats_down_world->points[i].x / filter_size_map) * filter_size_map + 0.5 * filter_size_map;
      mid_point.y =
        std::floor(feats_down_world->points[i].y / filter_size_map) * filter_size_map + 0.5 * filter_size_map;
      mid_point.z =
        std::floor(feats_down_world->points[i].z / filter_size_map) * filter_size_map + 0.5 * filter_size_map;

      const float dist = calc_dist(feats_down_world->points[i], mid_point);
      if (
        std::fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map &&
        std::fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map &&
        std::fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map) {
        point_no_need_downsample.push_back(feats_down_world->points[i]);
        continue;
      }
      for (int readd_i = 0; readd_i < num_match_points; ++readd_i) {
        if (static_cast<int>(points_near.size()) < num_match_points) break;
        if (calc_dist(points_near[readd_i], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        point_to_add.push_back(feats_down_world->points[i]);
      }
    } else {
      point_to_add.push_back(feats_down_world->points[i]);
    }
  }

  ikdtree.Add_Points(point_to_add, true);
  ikdtree.Add_Points(point_no_need_downsample, false);
}

float OdometryEstimation::calc_dist(const PointT& p1, const PointT& p2) {
  const float dx = p1.x - p2.x;
  const float dy = p1.y - p2.y;
  const float dz = p1.z - p2.z;
  return dx * dx + dy * dy + dz * dz;
}

thread_local OdometryEstimation* OdometryEstimation::active = nullptr;

}  // namespace fastlio
}  // namespace asuka
