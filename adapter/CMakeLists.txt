cmake_minimum_required(VERSION 3.16)
project(pi_vio_adapter LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgcodecs imgproc video calib3d features2d objdetect)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
set(VIO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")
add_executable(vio_replay replay.cpp)
add_executable(vio_stream stream.cpp)
foreach(adapter_target vio_replay vio_stream)
target_compile_definitions(${adapter_target} PRIVATE ROS_AVAILABLE=0)
target_include_directories(${adapter_target} PRIVATE ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  ${VIO_ROOT}/vendor/open_vins/ov_msckf/src
  ${VIO_ROOT}/vendor/open_vins/ov_core/src
  ${VIO_ROOT}/vendor/open_vins/ov_init/src)
target_link_libraries(${adapter_target} PRIVATE
  ${VIO_ROOT}/build/openvins/libov_msckf_lib.so ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})
target_compile_options(${adapter_target} PRIVATE -Wall -Wextra)
endforeach()
