cmake_minimum_required(VERSION 2.8.3)
project(calibration_test)

set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)
find_package(Eigen3 REQUIRED)
find_package( OpenCV REQUIRED )

catkin_package(

  DEPENDS Eigen3
)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  roscpp
  rospy
  sensor_msgs
  image_transport
  cmake_modules
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  
)

 add_executable(calibration_test_node ./src/calibration_test_node.cpp ./src/chess.cpp ./include/chess.h src/CalibData.cpp src/CalibData.h src/Calibration.cpp src/Frame.cpp src/Equation.cpp)

 add_dependencies(calibration_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

 target_link_libraries(calibration_test_node
   ${catkin_LIBRARIES}	
   ${Eigen_LIBRARIES}
 )

add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/tinyXML/tinyxml2.cpp)



