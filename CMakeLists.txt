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

option(BUILD_OPTIMIZATION_MODULES "Build optimization modules (requires GTSAM)." OFF)

# OpenCV is an unconditional dependency: the image input path
# (ImageData / on_insert_image / the executor's third queue) and the
# PointXYZIRGBOffset point type are always compiled in, and every frontend
# saves PCD maps in the color-capable PointXYZIRGB format.
find_package(OpenCV REQUIRED COMPONENTS core)

# Per-odometry switches: each frontend is dlopened at runtime via
# config_odometry's so_name, so any of them can be skipped at build time
# without touching the core. All default ON.
option(BUILD_ODOMETRY_FASTLIO "Build the asuka_odometry_fastlio odometry plugin." ON)
option(BUILD_ODOMETRY_SMALLPOINTLIO "Build the asuka_odometry_smallpointlio odometry plugin." ON)
option(BUILD_ODOMETRY_BATCHLIO "Build the asuka_odometry_batchlio odometry plugin." ON)
option(BUILD_ODOMETRY_SUPERLIO "Build the asuka_odometry_superlio odometry plugin (and its superlio_basic helper)." ON)
option(BUILD_ODOMETRY_FASTLIVO "Build the asuka_odometry_fastlivo odometry plugin." ON)
option(BUILD_ODOMETRY_SRLIVO "Build the asuka_odometry_srlivo odometry plugin." ON)
option(BUILD_ODOMETRY_ZLIO "Build the asuka_odometry_zlio odometry plugin (TDT z-localization frontend)." ON)

# libstdc++'s atomic shared_ptr free functions only take the lock-free path
# with cmpxchg16b (-mcx16): every TU that touches a CallbackSlot snapshot --
# asuka_core's own TUs and every linked consumer, standalone ones included --
# must use the same strategy, otherwise lock-free readers and lock-pool
# writers share one shared_ptr object in one process (undefined behavior).
# Applied as a PUBLIC option on asuka_core below: it covers this tree via
# target linkage and travels to installed consumers as
# INTERFACE_COMPILE_OPTIONS on the exported target.
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mcx16" ASUKA_HAS_MCX16)

add_definitions(-DPCL_NO_PRECOMPILE)

if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    find_package(catkin REQUIRED)
    catkin_package(
      INCLUDE_DIRS include thirdparty
      LIBRARIES asuka_core
    )
  endif()
endif()

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenMP)

if(BUILD_OPTIMIZATION_MODULES)
  find_package(GTSAM REQUIRED)
endif()

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set(asuka_LIBRARIES "")

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  thirdparty
)

add_library(asuka_ikd_tree STATIC include/asuka/odometry/fastlio/ikd_tree/ikd_tree.cpp)
# Its explicit KD_TREE<T> instantiations include asuka/core/types.hpp
# (PointXYZIRGBOffset), which needs the OpenCV headers.
target_include_directories(asuka_ikd_tree PUBLIC ${OpenCV_INCLUDE_DIRS})
set_target_properties(asuka_ikd_tree PROPERTIES POSITION_INDEPENDENT_CODE ON)



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
target_link_libraries(asuka_core PUBLIC ${OpenCV_LIBRARIES})
if(ASUKA_HAS_MCX16)
  target_compile_options(asuka_core PUBLIC -mcx16)
else()
  message(WARNING "asuka: -mcx16 is not supported; shared_ptr atomic operation fallback to libstdc++ lock pool implementation")
endif()
list(APPEND asuka_LIBRARIES asuka_core)

# asuka_odometry_fastlio
if(BUILD_ODOMETRY_FASTLIO)
  add_library(asuka_odometry_fastlio SHARED
    src/asuka/odometry/fastlio/odometry_estimation.cpp
    src/asuka/odometry/fastlio/imu_preprocess.cpp
    src/asuka/odometry/fastlio/cloud_preprocess.cpp
    src/asuka/odometry/fastlio/create.cpp
  )
  # OpenMP control for this frontend's two parallel loops (the process_scan
  # point-transform loop and the h_share_model residual loop share the
  # ENABLE_OMP / NUM_THREADS pair). Commented out by default: the pragma
  # regions compile out and the frontend runs serially. NUM_THREADS=1 keeps
  # the regions serial with identical results; when raising it consider
  # OMP_WAIT_POLICY=passive (libgomp spins between the frequent small
  # regions -- measured 46% parallel efficiency at 4 threads on the default
  # policy vs 77% passive).
  # target_compile_definitions(asuka_odometry_fastlio PRIVATE ENABLE_OMP NUM_THREADS=1)
  target_link_libraries(asuka_odometry_fastlio PUBLIC asuka_core)
  list(APPEND asuka_LIBRARIES asuka_odometry_fastlio)
endif()

if(BUILD_ODOMETRY_SMALLPOINTLIO)
  add_library(asuka_odometry_smallpointlio SHARED
    src/asuka/odometry/smallpointlio/odometry_estimation.cpp
    src/asuka/odometry/smallpointlio/imu_preprocess.cpp
    src/asuka/odometry/smallpointlio/cloud_preprocess.cpp
    src/asuka/odometry/smallpointlio/create.cpp
  )
  target_link_libraries(asuka_odometry_smallpointlio PUBLIC asuka_core)
  list(APPEND asuka_LIBRARIES asuka_odometry_smallpointlio)
endif()

if(BUILD_ODOMETRY_BATCHLIO)
  add_library(asuka_odometry_batchlio SHARED
    src/asuka/odometry/batchlio/odometry_estimation.cpp
    src/asuka/odometry/batchlio/imu_preprocess.cpp
    src/asuka/odometry/batchlio/cloud_preprocess.cpp
    src/asuka/odometry/batchlio/common.cpp
    src/asuka/odometry/batchlio/create.cpp
  )
    target_link_libraries(asuka_odometry_batchlio PUBLIC asuka_core)
  list(APPEND asuka_LIBRARIES asuka_odometry_batchlio)
endif()

# asuka_odometry_superlio (Super-LIO port)
if(BUILD_ODOMETRY_SUPERLIO)
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
  target_link_libraries(asuka_odometry_superlio PUBLIC asuka_core asuka_superlio_basic tbb ${PCL_LIBRARIES})
  list(APPEND asuka_LIBRARIES asuka_odometry_superlio)
endif()


# asuka_odometry_fastlivo (FAST-LIVO2 port). This frontend consumes asuka's
# image pipeline and carries the vendored old (non-template) Sophus plus the
# minimal vikut camera utilities it needs.
if(BUILD_ODOMETRY_FASTLIVO)
  add_library(asuka_fastlivo_basic STATIC
    thirdparty/vikit/src/pinhole_camera.cpp
    thirdparty/vikit/src/equidistant_camera.cpp
    thirdparty/vikit/src/math_utils.cpp
    thirdparty/vikit/src/vision.cpp
    thirdparty/vikit/src/robust_cost.cpp
  )
  # vikut headers resolve here; Sophus comes from the shared header-only
  # thirdparty/sophus (template API) already on the global include path.
  target_include_directories(asuka_fastlivo_basic PRIVATE
    thirdparty/vikit/include
    ${OpenCV_INCLUDE_DIRS}
  )
  set_target_properties(asuka_fastlivo_basic PROPERTIES POSITION_INDEPENDENT_CODE ON)
  # The vendored camera models call cv::initUndistortRectifyMap at
  # construction -- a calib3d symbol, beyond the core component asuka_core
  # uses (calib3d transitively brings imgproc).
  find_package(OpenCV REQUIRED COMPONENTS calib3d)
  target_link_libraries(asuka_fastlivo_basic PUBLIC ${OpenCV_LIBRARIES})

  add_library(asuka_odometry_fastlivo SHARED
    src/asuka/odometry/fastlivo/odometry_estimation.cpp
    src/asuka/odometry/fastlivo/imu_preprocess.cpp
    src/asuka/odometry/fastlivo/cloud_preprocess.cpp
    src/asuka/odometry/fastlivo/voxel_map.cpp
    src/asuka/odometry/fastlivo/vio.cpp
    src/asuka/odometry/fastlivo/frame.cpp
    src/asuka/odometry/fastlivo/visual_point.cpp
    src/asuka/odometry/fastlivo/create.cpp
  )
  target_include_directories(asuka_odometry_fastlivo PRIVATE
    thirdparty/vikit/include
  )
  # OpenMP control for the voxel-map covariance query loop and the VIO
  # photometric residual loop (ENABLE_OMP / NUM_THREADS, same unified knob
  # as fastlio). Commented out by default: the frontend runs serially.
  # target_compile_definitions(asuka_odometry_fastlivo PRIVATE ENABLE_OMP NUM_THREADS=1)
  target_link_libraries(asuka_odometry_fastlivo PUBLIC asuka_core asuka_fastlivo_basic)
  list(APPEND asuka_LIBRARIES asuka_fastlivo_basic asuka_odometry_fastlivo)
endif()

# asuka_odometry_srlivo (sr_livo port): CT-ICP odometry with the R3LIVE
# visual rendering branch. Images drive its measurement windows and the
# vision pipeline needs calib3d (solvePnPRansac / findFundamentalMat /
# initUndistortRectifyMap) beyond the core component asuka_core links.
if(BUILD_ODOMETRY_SRLIVO)
  find_package(OpenCV REQUIRED COMPONENTS core calib3d)

  add_library(asuka_odometry_srlivo SHARED
    src/asuka/odometry/srlivo/odometry_estimation.cpp
    src/asuka/odometry/srlivo/optimize.cpp
    src/asuka/odometry/srlivo/frame.cpp
    src/asuka/odometry/srlivo/cloud_preprocess.cpp
    src/asuka/odometry/srlivo/eskf_estimator.cpp
    src/asuka/odometry/srlivo/image_processing.cpp
    src/asuka/odometry/srlivo/optical_flow_tracker.cpp
    src/asuka/odometry/srlivo/rgb_map_tracker.cpp
    src/asuka/odometry/srlivo/cloud_map.cpp
    src/asuka/odometry/srlivo/state.cpp
    src/asuka/odometry/srlivo/utility.cpp
    src/asuka/odometry/srlivo/lk_pyramid.cpp
    src/asuka/odometry/srlivo/create.cpp
  )
  # its vendored tsl/robin_map headers resolve via the module-local include
  target_include_directories(asuka_odometry_srlivo PRIVATE include/asuka/odometry/srlivo)
  target_link_libraries(asuka_odometry_srlivo PUBLIC asuka_core ${OpenCV_LIBRARIES})
  list(APPEND asuka_LIBRARIES asuka_odometry_srlivo)
endif()

# asuka_odometry_zlio (TDT z-localization frontend): 15-dim semi-direct product
# error-state filter over a continuous, adaptively windowed point stream. The
# vendored libzlio core is ROS-free upstream; yaml-cpp was replaced by asuka's
# config system (src/asuka/odometry/zlio/config.cpp read layer) and nanoflann is
# vendored next to the module (registration.cpp is its only consumer).
if(BUILD_ODOMETRY_ZLIO)
  add_library(asuka_odometry_zlio SHARED
    src/asuka/odometry/zlio/config.cpp
    src/asuka/odometry/zlio/execution.cpp
    src/asuka/odometry/zlio/filter.cpp
    src/asuka/odometry/zlio/geometry.cpp
    src/asuka/odometry/zlio/inertial.cpp
    src/asuka/odometry/zlio/initialization.cpp
    src/asuka/odometry/zlio/input.cpp
    src/asuka/odometry/zlio/map.cpp
    src/asuka/odometry/zlio/observation.cpp
    src/asuka/odometry/zlio/odometry.cpp
    src/asuka/odometry/zlio/plane.cpp
    src/asuka/odometry/zlio/prediction.cpp
    src/asuka/odometry/zlio/preintegration.cpp
    src/asuka/odometry/zlio/registration.cpp
    src/asuka/odometry/zlio/sampling.cpp
    src/asuka/odometry/zlio/state.cpp
    src/asuka/odometry/zlio/trajectory.cpp
    src/asuka/odometry/zlio/uncertainty.cpp
    src/asuka/odometry/zlio/window.cpp
    src/asuka/odometry/zlio/cloud_preprocess.cpp
    src/asuka/odometry/zlio/odometry_estimation.cpp
    src/asuka/odometry/zlio/create.cpp
  )
  target_include_directories(asuka_odometry_zlio PRIVATE include/asuka/odometry/zlio)
  target_link_libraries(asuka_odometry_zlio PUBLIC asuka_core)
  list(APPEND asuka_LIBRARIES asuka_odometry_zlio)
endif()

# zlio regression tests (ported from upstream gtest suites; the yaml config
# loader call in initialization_test was replaced by the default Config whose
# values match config_zlio.json). Pure-algorithm, no ROS.
if(BUILD_ODOMETRY_ZLIO)
  find_package(GTest QUIET)
  if(GTEST_FOUND)
    enable_testing()
    foreach(zlio_test state_test input_test initialization_test)
      add_executable(asuka_zlio_${zlio_test} src/asuka/odometry/zlio/tests/${zlio_test}.cpp)
      target_include_directories(asuka_zlio_${zlio_test} PRIVATE include/asuka/odometry/zlio)
      target_link_libraries(asuka_zlio_${zlio_test} asuka_core asuka_odometry_zlio GTest::GTest GTest::Main pthread)
      add_test(NAME zlio_${zlio_test} COMMAND asuka_zlio_${zlio_test})
    endforeach()
  else()
    message(WARNING "GTest not found: zlio regression tests skipped")
  endif()
endif()

# imu_prediction: a pure-algorithm extension module (no ROS dependency) --
# it consumes the process-wide callback slots and emits high-rate odometry.
add_library(asuka_imu_prediction SHARED src/asuka/modules/imu_prediction.cpp)
target_link_libraries(asuka_imu_prediction asuka_core)
list(APPEND asuka_LIBRARIES asuka_imu_prediction)

if(BUILD_OPTIMIZATION_MODULES)
  add_library(asuka_scancontext STATIC
    thirdparty/scancontext/Scancontext.cpp
  )
  target_include_directories(asuka_scancontext PUBLIC
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    thirdparty/scancontext
    ${OpenCV_INCLUDE_DIRS}
  )
  set_target_properties(asuka_scancontext PROPERTIES POSITION_INDEPENDENT_CODE ON)

  add_library(asuka_optimization_loam SHARED src/asuka/modules/optimization_loam.cpp)
  target_link_libraries(asuka_optimization_loam asuka_core asuka_scancontext gtsam)
  list(APPEND asuka_LIBRARIES asuka_optimization_loam)
endif()


# Everything installable, including the internal static helpers that are
# absorbed into the shared libraries at link time.
# asuka_ikd_tree is the only installable helper outside asuka_LIBRARIES -- a
# second mention made install(EXPORT) fail with "includes target more than once".
set(asuka_INSTALL_TARGETS ${asuka_LIBRARIES} asuka_ikd_tree)
if(BUILD_ODOMETRY_SUPERLIO)
  list(APPEND asuka_INSTALL_TARGETS asuka_superlio_basic)
endif()
if(BUILD_OPTIMIZATION_MODULES)
  list(APPEND asuka_INSTALL_TARGETS asuka_scancontext)
endif()

#############
## Install ##
#############
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    find_package(catkin REQUIRED)
    # spdlog is listed so downstream ${asuka_LIBRARIES}-style consumers pick it
    # up transitively (catkin does not propagate imported targets otherwise).
    catkin_package(
      INCLUDE_DIRS include thirdparty
      LIBRARIES ${asuka_LIBRARIES} spdlog::spdlog
    )
    install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    # nlohmann is part of the public interface (config_impl.hpp includes it);
    # the rest of thirdparty stays private to this package.
    install(DIRECTORY thirdparty/nlohmann DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    install(TARGETS ${asuka_INSTALL_TARGETS}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  endif()
else()
  # Standalone build (no ROS environment): plain CMake install with exported
  # targets and a config file, mirror of the glim layout.
  include(GNUInstallDirs)
  set(ASUKA_CMAKE_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/asuka"
    CACHE PATH "Install directory for the asuka CMake config files")

  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(TARGETS ${asuka_INSTALL_TARGETS}
    EXPORT asuka-targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
  install(EXPORT asuka-targets
    FILE asuka-targets.cmake
    NAMESPACE asuka::
    DESTINATION ${ASUKA_CMAKE_INSTALL_DIR}
  )
  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/asuka-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/asuka-config.cmake"
    INSTALL_DESTINATION ${ASUKA_CMAKE_INSTALL_DIR}
  )
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/asuka-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
  )
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/asuka-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/asuka-config-version.cmake"
    DESTINATION ${ASUKA_CMAKE_INSTALL_DIR}
  )
endif()
