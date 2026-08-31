//
// Created by xiang on 25-3-12.
//

#pragma once
#ifndef LIGHTNING_POINT_DEF_H
#define LIGHTNING_POINT_DEF_H

#include "common/eigen_types.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace lightning {


using PointType = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

inline Vec3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }

using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
constexpr double G_m_s2 = 9.81;  // Gravity const in GuangDong/China

}  // namespace lightning

#endif  // LIGHTNING_POINT_DEF_H
