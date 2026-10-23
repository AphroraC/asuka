#include <asuka/modules/srlivo/frame.hpp>

#include <cmath>

#include <asuka/modules/srlivo/state.hpp>

namespace asuka {
namespace srlivo {

cloudFrame::cloudFrame(std::vector<point3D> &point_frame_, state *p_state_) {
  point_frame.insert(point_frame.end(), point_frame_.begin(), point_frame_.end());

  p_state = p_state_;
}

cloudFrame::cloudFrame(cloudFrame *p_cloud_frame) {
  time_sweep_begin = p_cloud_frame->time_sweep_begin;
  time_sweep_end = p_cloud_frame->time_sweep_end;
  time_frame_begin = p_cloud_frame->time_frame_begin;
  time_frame_end = p_cloud_frame->time_frame_end;

  id = p_cloud_frame->id;
  sub_id = p_cloud_frame->sub_id;
  frame_id = p_cloud_frame->frame_id;

  p_state = p_cloud_frame->p_state;

  point_frame.insert(point_frame.end(), p_cloud_frame->point_frame.begin(), p_cloud_frame->point_frame.end());

  offset_begin = p_cloud_frame->offset_begin;
  offset_end = p_cloud_frame->offset_end;
  dt_offset = p_cloud_frame->dt_offset;
}

void cloudFrame::release() {
  std::vector<point3D>().swap(point_frame);

  if (p_state != nullptr)
    p_state->release();

  delete p_state;

  p_state = nullptr;

  if (!rgb_image.empty())
    rgb_image.release();

  if (!gray_image.empty())
    gray_image.release();

  image_data.reset();
}

bool cloudFrame::if2dPointsAvailable(const double &u, const double &v, const double &scale, double fov_mar) {
  double used_fov_margin = p_state->fov_margin;

  if (fov_mar > 0.0)
    used_fov_margin = fov_mar;

  if ((u / scale >= (used_fov_margin * image_cols + 1)) && (std::ceil(u / scale) < ((1 - used_fov_margin) * image_cols)) &&
      (v / scale >= (used_fov_margin * image_rows + 1)) && (std::ceil(v / scale) < ((1 - used_fov_margin) * image_rows)))
    return true;
  else
    return false;
}

bool cloudFrame::getRgb(const double &u, const double &v, int &r, int &g, int &b) {
  r = rgb_image.at<cv::Vec3b>(v, u)[2];
  g = rgb_image.at<cv::Vec3b>(v, u)[1];
  b = rgb_image.at<cv::Vec3b>(v, u)[0];

  return true;
}

template <typename T>
inline T getSubPixel(cv::Mat &mat, const double &row, const double &col, double pyramid_layer = 0) {
  int floor_row = floor(row);
  int floor_col = floor(col);

  double frac_row = row - floor_row;
  double frac_col = col - floor_col;

  int ceil_row = floor_row + 1;
  int ceil_col = floor_col + 1;

  if (pyramid_layer != 0) {
    int pos_bias = pow(2, pyramid_layer - 1);

    floor_row -= pos_bias;
    floor_col -= pos_bias;
    ceil_row += pos_bias;
    ceil_row += pos_bias;
  }

  return ((1.0 - frac_row) * (1.0 - frac_col) * (T)mat.ptr<T>(floor_row)[floor_col]) +
         (frac_row * (1.0 - frac_col) * (T)mat.ptr<T>(ceil_row)[floor_col]) +
         ((1.0 - frac_row) * frac_col * (T)mat.ptr<T>(floor_row)[ceil_col]) +
         (frac_row * frac_col * (T)mat.ptr<T>(ceil_row)[ceil_col]);
}

Eigen::Vector3d cloudFrame::getRgb(double &u, double &v, int layer, Eigen::Vector3d *rgb_dx, Eigen::Vector3d *rgb_dy) {
  const int ssd = 5;

  cv::Vec3b rgb = getSubPixel<cv::Vec3b>(rgb_image, v, u, layer);

  if (rgb_dx != nullptr) {
    cv::Vec3f rgb_left(0, 0, 0), rgb_right(0, 0, 0);

    float pixel_dif = 0;

    for (int bias_idx = 1; bias_idx < ssd; bias_idx++) {
      rgb_left += getSubPixel<cv::Vec3b>(rgb_image, v, u - bias_idx, layer);
      rgb_right += getSubPixel<cv::Vec3b>(rgb_image, v, u + bias_idx, layer);
      pixel_dif += 2 * bias_idx;
    }

    cv::Vec3f cv_rgb_dx = rgb_right - rgb_left;
    *rgb_dx = Eigen::Vector3d(cv_rgb_dx(0), cv_rgb_dx(1), cv_rgb_dx(2)) / pixel_dif;
  }

  if (rgb_dy != nullptr) {
    cv::Vec3f rgb_up(0, 0, 0), rgb_down(0, 0, 0);

    float pixel_dif = 0;

    for (int bias_idx = 1; bias_idx < ssd; bias_idx++) {
      rgb_up += getSubPixel<cv::Vec3b>(rgb_image, v - bias_idx, u, layer);
      rgb_down += getSubPixel<cv::Vec3b>(rgb_image, v + bias_idx, u, layer);
      pixel_dif += 2 * bias_idx;
    }

    cv::Vec3f cv_rgb_dy = rgb_down - rgb_up;
    *rgb_dy = Eigen::Vector3d(cv_rgb_dy(0), cv_rgb_dy(1), cv_rgb_dy(2)) / pixel_dif;
  }

  return Eigen::Vector3d(rgb[0], rgb[1], rgb[2]);
}

bool cloudFrame::project3dTo2d(const pcl::PointXYZI &point_in, double &u, double &v, const double &scale) {
  Eigen::Vector3d point_world(point_in.x, point_in.y, point_in.z);

  Eigen::Vector3d point_camera = p_state->q_camera_world.toRotationMatrix() * point_world + p_state->t_camera_world;

  if (point_camera(2, 0) < 0.001) {
    return false;
  }

  u = (point_camera(0) * p_state->fx / point_camera(2) + p_state->cx) * scale;
  v = (point_camera(1) * p_state->fy / point_camera(2) + p_state->cy) * scale;

  return true;
}

bool cloudFrame::project3dPointInThisImage(const pcl::PointXYZI &point_in, double &u, double &v, pcl::PointXYZRGB *rgb_point, double intrinsic_scale) {
  if (project3dTo2d(point_in, u, v, intrinsic_scale) == false)
    return false;

  if (if2dPointsAvailable(u, v, intrinsic_scale) == false)
    return false;

  if (rgb_point != nullptr) {
    int r = 0;
    int g = 0;
    int b = 0;
    getRgb(u, v, r, g, b);
    rgb_point->x = point_in.x;
    rgb_point->y = point_in.y;
    rgb_point->z = point_in.z;
    rgb_point->r = r;
    rgb_point->g = g;
    rgb_point->b = b;
    rgb_point->a = 255;
  }

  return true;
}

bool cloudFrame::project3dPointInThisImage(const Eigen::Vector3d &point_in, double &u, double &v, pcl::PointXYZRGB *rgb_point, double intrinsic_scale) {
  pcl::PointXYZI temp_point;
  temp_point.x = point_in(0);
  temp_point.y = point_in(1);
  temp_point.z = point_in(2);

  return project3dPointInThisImage(temp_point, u, v, rgb_point, intrinsic_scale);
}

void cloudFrame::refreshPoseForProjection() {
  p_state->q_camera_world = p_state->q_world_camera.inverse();
  p_state->t_camera_world = -p_state->q_camera_world.toRotationMatrix() * p_state->t_world_camera;
}

estimationSummary::estimationSummary() {
}

void estimationSummary::release() {
}

}  // namespace srlivo
}  // namespace asuka
