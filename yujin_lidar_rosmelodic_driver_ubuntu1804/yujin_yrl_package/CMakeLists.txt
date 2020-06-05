cmake_minimum_required(VERSION 2.8.3)
project(yujin_yrl_package)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  tf
)

#add_message_files(FILES PointCloud_Msg.msg
#generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
LIBRARIES yujin_yrl_package
CATKIN_DEPENDS std_msgs roscpp sensor_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)
include_directories(../libyrl_library/include/yrl_library)
link_directories(../libyrl_library/lib/)

add_executable(yrl_pub src/yrl_pub.cpp)
add_dependencies(yrl_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(yrl_pub ${catkin_LIBRARIES} pthread rt dl)

