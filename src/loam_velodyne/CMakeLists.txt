cmake_minimum_required(VERSION 2.8.3)
project(loam_velodyne)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  tf)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(libpointmatcher REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

add_message_files(
        DIRECTORY msg
        FILES
)

generate_messages(
        DEPENDENCIES
        sensor_msgs
        std_msgs
)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS} src)

catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)

# 添加g2o的依赖
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH /home/lx/lx_slam/src/g2o-master/cmake_modules )
SET( G2O_ROOT /usr/local/include/g2o )
FIND_PACKAGE( G2O )
include_directories(
        ${G2O_INCLUDE_DIRS}
        "/usr/include/eigen3"
)
# CSparse
#FIND_PACKAGE( CSparse )
#INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")

add_library(segmenter src/segmenter.cpp src/segmenter.h src/common_function.h)
target_link_libraries(segmenter ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS})

#add_library(graphSlam src/graphSlam.cpp src/graphSlam.h include/g2o/edge_se3_plane.h src/common_function.h
#        include/g2o/edge_se3_priorxy.h include/g2o/edge_se3_priorxyz.h)

#target_link_libraries(graphSlam ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS} ${G2O_LIBS} ${G2O_CORE_LIBRARY}
#        ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_PCG} ${G2O_SOLVER_CSPARSE} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM3D_ADDONS}
#        ${G2O_TYPES_DATA})

# ${G2O_LIBS}
add_library(Odometry src/Odometry.cpp src/Odometry.h src/common_function.h)
target_link_libraries(Odometry ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS})

add_executable(scanRegistration src/scanRegistration.cpp)
target_link_libraries(scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserOdometry src/laserOdometry.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(laserMapping src/laserMapping.cpp)
add_dependencies(laserMapping ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} tinyxml)

add_executable(transformMaintenance src/transformMaintenance.cpp )
add_dependencies(transformMaintenance ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} tinyxml Odometry segmenter)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  # TODO: Download test data
  catkin_download_test_data(${PROJECT_NAME}_test_data.tar.gz
    https://dl.dropboxusercontent.com/s/y4hn486461tfmpm/velodyne_loam_test_data.tar.gz
    MD5 3d5194e6981975588b7a93caebf79ba4)
  add_custom_target(${PROJECT_NAME}_test_data
    COMMAND ${CMAKE_COMMAND} -E tar -xzf velodyne_loam_test_data.tar.gz
    DEPENDS ${PROJECT_NAME}_test_data.tar.gz)
  configure_file(tests/loam.test.in
                 ${PROJECT_BINARY_DIR}/test/loam.test)
  add_rostest(${PROJECT_BINARY_DIR}/test/loam.test
    DEPENDENCIES
      ${PROJECT_NAME}_test_data
      scanRegistration
      laserOdometry
      laserMapping
      transformMaintenance
)
endif()


