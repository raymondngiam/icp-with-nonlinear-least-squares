cmake_minimum_required(VERSION 3.19)

project(ICP_bunny)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)
find_package(Boost REQUIRED)

add_executable(main main.cpp)
target_link_libraries (main Eigen3::Eigen Ceres::ceres fmt::fmt Boost::boost Sophus::Sophus)
