cmake_minimum_required(VERSION 3.0.2)
project(asuka VERSION 1.0.0 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    message(FATAL_ERROR "GCC >= 9 is required for std::filesystem support. "
                        "Current version: ${CMAKE_CXX_COMPILER_VERSION}")
  endif()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type." FORCE)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcx16" ASUKA_HAS_MCX16)
if(ASUKA_HAS_MCX16)
  add_compile_options(-mcx16)
else()
  message(WARNING "asuka: -mcx16 is not supported; shared_ptr atomic operation fallback to libstdc++ lock pool implementation")
endif()

# Enable OpenMP in FastLIO, but single thread is better
# add_definitions(-DFASTLIO_ENABLE_OMP -DFASTLIO_NUM_THREADS=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  pcl_ros
  pcl_conversions
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(spdlog REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP)

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES asuka_core
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  thirdparty
)

add_library(asuka_ikd_tree STATIC thirdparty/ikd_tree/ikd_tree.cpp)
target_include_directories(asuka_ikd_tree PUBLIC thirdparty)
set_target_properties(asuka_ikd_tree PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(asuka_miao STATIC
  thirdparty/miao/core/graph/graph.cc
  thirdparty/miao/core/graph/optimizer.cc
  thirdparty/miao/core/common/string_tools.cc
  thirdparty/miao/core/math/marginal_covariance_cholesky.cc
  thirdparty/miao/core/opti_algo/optimization_algorithm.cc
  thirdparty/miao/core/opti_algo/opti_algo_gauss_newton.cc
  thirdparty/miao/core/opti_algo/opti_algo_lm.cc
  thirdparty/miao/core/opti_algo/opti_algo_dogleg.cc
  thirdparty/miao/core/solver/solver.cc
)
target_include_directories(asuka_miao PUBLIC
  ${Eigen3_INCLUDE_DIRS}
  thirdparty/miao
  thirdparty/lightning
  thirdparty/sophus 
  )
set_target_properties(asuka_miao PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(asuka_miao PUBLIC glog tbb)

add_library(asuka_core SHARED
  src/asuka/utility/config.cpp
  src/asuka/utility/logging.cpp
  src/asuka/utility/extension_module.cpp
  src/asuka/utility/load_module.cpp
  src/asuka/core/async_odometry_estimation.cpp
  src/asuka/core/callbacks.cpp
  src/asuka/core/odometry_estimation.cpp
)
target_include_directories(asuka_core PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  thirdparty
)
target_link_libraries(asuka_core PUBLIC
  ${PCL_LIBRARIES}
  spdlog::spdlog
  asuka_ikd_tree
)

# asuka_odometry_fastlio
add_library(asuka_odometry_fastlio SHARED
  src/asuka/odometry/fastlio/odometry_estimation.cpp
  src/asuka/odometry/fastlio/imu_preprocess.cpp
  src/asuka/odometry/fastlio/cloud_preprocess.cpp
  src/asuka/odometry/fastlio/create.cpp
)
target_link_libraries(asuka_odometry_fastlio PUBLIC asuka_core)

add_library(asuka_odometry_smallpointlio SHARED
  src/asuka/odometry/smallpointlio/odometry_estimation.cpp
  src/asuka/odometry/smallpointlio/imu_preprocess.cpp
  src/asuka/odometry/smallpointlio/cloud_preprocess.cpp
  src/asuka/odometry/smallpointlio/create.cpp
)
target_link_libraries(asuka_odometry_smallpointlio PUBLIC asuka_core)

add_library(asuka_odometry_batchlio SHARED
  src/asuka/odometry/batchlio/odometry_estimation.cpp
  src/asuka/odometry/batchlio/imu_preprocess.cpp
  src/asuka/odometry/batchlio/cloud_preprocess.cpp
  src/asuka/odometry/batchlio/common.cpp
  src/asuka/odometry/batchlio/create.cpp
)
target_include_directories(asuka_odometry_batchlio PRIVATE thirdparty/pointlio)
target_link_libraries(asuka_odometry_batchlio PUBLIC asuka_core)

# asuka_odometry_superlio (Super-LIO port)
add_library(asuka_superlio_basic STATIC
  src/asuka/odometry/superlio/manifold.cpp
)
target_include_directories(asuka_superlio_basic PUBLIC include ${Eigen3_INCLUDE_DIRS})
set_target_properties(asuka_superlio_basic PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(asuka_odometry_superlio SHARED
  src/asuka/odometry/superlio/odometry_estimation.cpp
  src/asuka/odometry/superlio/eskf.cpp
  src/asuka/odometry/superlio/imu_preprocess.cpp
  src/asuka/odometry/superlio/cloud_preprocess.cpp
  src/asuka/odometry/superlio/create.cpp
)
target_include_directories(asuka_odometry_superlio PRIVATE thirdparty/superlio)
target_link_libraries(asuka_odometry_superlio PUBLIC asuka_core asuka_superlio_basic tbb ${PCL_LIBRARIES})

# asuka_odometry_lightning (AAFasterLIO port)
add_library(asuka_odometry_lightning SHARED
  src/asuka/odometry/lightning/odometry_estimation.cpp
  src/asuka/odometry/lightning/imu_preprocess.cpp
  src/asuka/odometry/lightning/cloud_preprocess.cpp
  src/asuka/odometry/lightning/create.cpp
  thirdparty/lightning/common/nav_state.cc
  thirdparty/lightning/common/options.cc
  thirdparty/lightning/utils/timer.cc
  thirdparty/lightning/core/lio/eskf.cc
)
target_include_directories(asuka_odometry_lightning PRIVATE thirdparty/lightning thirdparty/sophus)
target_link_libraries(asuka_odometry_lightning PUBLIC asuka_core glog tbb ${PCL_LIBRARIES})

add_library(asuka_scancontext STATIC
  thirdparty/scancontext/Scancontext.cpp
)
target_include_directories(asuka_scancontext PUBLIC
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  thirdparty/scancontext
)
set_target_properties(asuka_scancontext PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(asuka_ros SHARED src/asuka/asuka_ros.cpp)
target_link_libraries(asuka_ros asuka_core ${catkin_LIBRARIES})

add_library(asuka_rviz_viewer SHARED src/asuka/modules/rviz_viewer.cpp)
target_link_libraries(asuka_rviz_viewer asuka_core ${catkin_LIBRARIES})
add_library(asuka_imu_prediction SHARED src/asuka/modules/imu_prediction.cpp)
target_link_libraries(asuka_imu_prediction asuka_core)
add_library(asuka_optimization_loam SHARED src/asuka/modules/optimization_loam.cpp)
target_link_libraries(asuka_optimization_loam asuka_core asuka_scancontext gtsam)
add_library(asuka_optimization_miao SHARED src/asuka/modules/optimization_miao.cpp)
target_link_libraries(asuka_optimization_miao asuka_core asuka_miao ${PCL_LIBRARIES})

add_library(asuka_nodelet apps/asuka_nodelet.cpp)
target_link_libraries(asuka_nodelet asuka_ros ${catkin_LIBRARIES})
add_executable(asuka_rosnode apps/asuka_rosnode.cpp)
target_link_libraries(asuka_rosnode asuka_ros ${catkin_LIBRARIES})
add_executable(asuka_rosbag apps/asuka_rosbag.cpp)
target_link_libraries(asuka_rosbag asuka_ros ${catkin_LIBRARIES})

install(TARGETS asuka_core asuka_ikd_tree asuka_miao asuka_scancontext asuka_odometry_fastlio asuka_odometry_batchlio
  asuka_odometry_smallpointlio asuka_odometry_lightning asuka_odometry_superlio asuka_superlio_basic
  asuka_ros asuka_rviz_viewer asuka_imu_prediction asuka_optimization_loam asuka_optimization_miao
  asuka_nodelet asuka_rosnode asuka_rosbag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
