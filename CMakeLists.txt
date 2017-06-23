cmake_minimum_required(VERSION 2.4.6)
project(asr_ism_visualizations)

add_definitions(-Wall -std=gnu++0x -ggdb -lboost_system)

find_package(cmake_modules)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  asr_msgs
  visualization_msgs 
  asr_lib_ism
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/ism_result_viz.cfg
  cfg/ism_pose_prediction_viz.cfg
  cfg/rp_pose_prediction_viz.cfg
  cfg/ism_voting_viz.cfg
  cfg/ism_valid_position_viz.cfg

)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp tf asr_msgs visualization_msgs asr_lib_ism
	DEPENDS EIGEN3 Boost
)

FILE(GLOB_RECURSE HEADERS "include/asr_ism_visualizations/*.hpp")
add_library(asr_ism_visualizations ${HEADERS}
			       src/ism_result_visualizer_rviz.cpp
                               src/old_result_visualizer_rviz.cpp
                               src/visualizer_rviz.cpp
                               src/PosePredictionVisualizerRVIZ.cpp
                               src/ism_pose_prediction_visualizer_rviz.cpp
                               src/ism_voting_visualizer_rviz.cpp
                               src/ObjectModelVisualizerRVIZ.cpp
                               src/record_visualizer_rviz.cpp
                               src/model_visualizer_rviz.cpp
                               src/vote_visualizer_rviz.cpp
                               src/VizHelperRVIZ.cpp
)

add_dependencies(asr_ism_visualizations
    asr_msgs_gencpp
    ${PROJECT_NAME}_gencfg
)
