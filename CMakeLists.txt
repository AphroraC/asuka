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

find_package(OpenCV REQUIRED COMPONENTS core)

option(BUILD_ODOMETRY_SUPERLIO "Build the asuka_odometry_superlio odometry plugin." ON)
option(BUILD_ODOMETRY_FASTLIO "Build the asuka_odometry_fastlio odometry plugin." ON)
option(BUILD_ODOMETRY_FASTLIVO "Build the asuka_odometry_fastlivo odometry plugin." ON)

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

add_library(asuka_ikd_tree STATIC include/asuka/modules/fastlio/ikd_tree/ikd_tree.cpp)
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

if(BUILD_ODOMETRY_FASTLIO)
  add_library(asuka_odometry_fastlio SHARED
    src/asuka/modules/fastlio/odometry_estimation.cpp
    src/asuka/modules/fastlio/imu_preprocess.cpp
    src/asuka/modules/fastlio/cloud_preprocess.cpp
    src/asuka/modules/fastlio/create.cpp
  )
  target_link_libraries(asuka_odometry_fastlio PUBLIC asuka_core)
  # target_compile_definitions(asuka_odometry_fastlio PRIVATE ENABLE_OMP NUM_THREADS=4)
  list(APPEND asuka_LIBRARIES asuka_odometry_fastlio)
endif()

if(BUILD_ODOMETRY_SUPERLIO)
  add_library(asuka_superlio_basic STATIC
    src/asuka/modules/superlio/manifold.cpp
  )
  target_include_directories(asuka_superlio_basic PUBLIC include ${Eigen3_INCLUDE_DIRS})
  set_target_properties(asuka_superlio_basic PROPERTIES POSITION_INDEPENDENT_CODE ON)

  add_library(asuka_odometry_superlio SHARED
    src/asuka/modules/superlio/odometry_estimation.cpp
    src/asuka/modules/superlio/eskf.cpp
    src/asuka/modules/superlio/imu_preprocess.cpp
    src/asuka/modules/superlio/cloud_preprocess.cpp
    src/asuka/modules/superlio/create.cpp
  )
  target_link_libraries(asuka_odometry_superlio PUBLIC asuka_core asuka_superlio_basic ${PCL_LIBRARIES})
  # target_compile_definitions(asuka_odometry_superlio PRIVATE ENABLE_OMP NUM_THREADS=4)
  list(APPEND asuka_LIBRARIES asuka_odometry_superlio)
endif()

if(BUILD_ODOMETRY_FASTLIVO)
  add_library(asuka_fastlivo_basic STATIC
    thirdparty/vikit/src/pinhole_camera.cpp
    thirdparty/vikit/src/equidistant_camera.cpp
    thirdparty/vikit/src/math_utils.cpp
    thirdparty/vikit/src/vision.cpp
    thirdparty/vikit/src/robust_cost.cpp
  )
  target_include_directories(asuka_fastlivo_basic PRIVATE
    thirdparty/vikit/include
    ${OpenCV_INCLUDE_DIRS}
  )
  set_target_properties(asuka_fastlivo_basic PROPERTIES POSITION_INDEPENDENT_CODE ON)
  find_package(OpenCV REQUIRED COMPONENTS calib3d)
  target_link_libraries(asuka_fastlivo_basic PUBLIC ${OpenCV_LIBRARIES})

  add_library(asuka_odometry_fastlivo SHARED
    src/asuka/modules/fastlivo/odometry_estimation.cpp
    src/asuka/modules/fastlivo/imu_preprocess.cpp
    src/asuka/modules/fastlivo/cloud_preprocess.cpp
    src/asuka/modules/fastlivo/voxel_map.cpp
    src/asuka/modules/fastlivo/vio.cpp
    src/asuka/modules/fastlivo/frame.cpp
    src/asuka/modules/fastlivo/visual_point.cpp
    src/asuka/modules/fastlivo/create.cpp
  )
  target_include_directories(asuka_odometry_fastlivo PRIVATE
    thirdparty/vikit/include
  )
  target_link_libraries(asuka_odometry_fastlivo PUBLIC asuka_core asuka_fastlivo_basic)
  # target_compile_definitions(asuka_odometry_fastlivo PRIVATE ENABLE_OMP NUM_THREADS=4)
  list(APPEND asuka_LIBRARIES asuka_fastlivo_basic asuka_odometry_fastlivo)
endif()

set(asuka_INSTALL_TARGETS ${asuka_LIBRARIES} asuka_ikd_tree)
if(BUILD_ODOMETRY_SUPERLIO)
  list(APPEND asuka_INSTALL_TARGETS asuka_superlio_basic)
endif()
#############
## Install ##
#############
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 1)
    find_package(catkin REQUIRED)
    catkin_package(
      INCLUDE_DIRS include thirdparty
      LIBRARIES ${asuka_LIBRARIES} spdlog::spdlog
    )
    install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    install(DIRECTORY thirdparty/nlohmann DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
    install(TARGETS ${asuka_INSTALL_TARGETS}
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  endif()
else()
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
