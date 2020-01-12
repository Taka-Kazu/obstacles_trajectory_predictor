cmake_minimum_required(VERSION 2.8.3)
project(obstacles_trajectory_predictor)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf2
    tf2_ros
)

find_package(Eigen3 REQUIRED COMPONENTS system)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES libobstacles_trajectory_predictor
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
)

add_library(libobstacles_trajectory_predictor
    src/obstacles_trajectory_predictor.cpp
    src/obstacles_tracker.cpp
    src/obstacle.cpp
    src/social_force_model.cpp
)
add_dependencies(libobstacles_trajectory_predictor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(libobstacles_trajectory_predictor
    ${catkin_LIBRARIES}
)

add_executable(obstacles_trajectory_predictor_demo src/demo.cpp)
add_dependencies(obstacles_trajectory_predictor_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacles_trajectory_predictor_demo
    ${catkin_LIBRARIES}
    libobstacles_trajectory_predictor
)

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(obstacles_trajectory_predictor_test
        test/obstacles_trajectory_predictor.test
        test/obstacles_trajectory_predictor_test.cpp
    )
    target_link_libraries(obstacles_trajectory_predictor_test
        libobstacles_trajectory_predictor
        ${catkin_LIBRARIES}
    )
endif()
