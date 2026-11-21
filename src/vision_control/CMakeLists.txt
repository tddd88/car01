cmake_minimum_required(VERSION 3.0.2)
project(vision_control)
add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge diagnostic_msgs geometry_msgs image_transport roscpp sensor_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
find_package(OpenCV REQUIRED)
catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS
  cv_bridge diagnostic_msgs geometry_msgs image_transport roscpp sensor_msgs
  tf2 tf2_ros tf2_geometry_msgs
)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(blue_detector_node src/blue_detector_node.cpp)
target_link_libraries(blue_detector_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(visual_servo_node src/visual_servo_node.cpp)
target_link_libraries(visual_servo_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(servo_geometry_test test/servo_geometry_test.cpp)
  if(TARGET servo_geometry_test)
    target_link_libraries(servo_geometry_test ${catkin_LIBRARIES})
  endif()
  catkin_install_python(PROGRAMS test/test_vision_interfaces.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/vision_interfaces.test)
endif()

install(TARGETS blue_detector_node visual_servo_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
