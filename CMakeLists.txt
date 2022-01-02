cmake_minimum_required(VERSION 2.8)
project(atHome_Data_Parser)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

if(POLICY CMP0074)
        cmake_policy(SET CMP0074 NEW)
endif()

# OpenCV
find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.0 not found.")
   endif()
endif()

# Build Parser
add_executable(data_parser src/DataParser.cpp src/Common.cpp)
target_include_directories(data_parser PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(data_parser ${OpenCV_LIBS})

# Build Parser ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  cv_bridge
  message_filters
)

if(catkin_FOUND)
   add_executable(data_parser_ros src/DataParserROS.cpp src/Common.cpp)
   target_link_libraries(data_parser_ros PUBLIC ${OpenCV_LIBS} ${catkin_LIBRARIES})
   target_include_directories(data_parser_ros PUBLIC ${PROJECT_SOURCE_DIR}/src ${catkin_INCLUDE_DIRS})

   add_executable(data_viewer_ros src/DataViewerRos.cpp)
   target_link_libraries(data_viewer_ros PUBLIC ${OpenCV_LIBS} ${catkin_LIBRARIES})
   target_include_directories(data_viewer_ros PUBLIC ${PROJECT_SOURCE_DIR}/src ${catkin_INCLUDE_DIRS})
endif()