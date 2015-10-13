cmake_minimum_required(VERSION 2.8.3)
project(bot_pattern_formation)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs image_transport vrep_common)

include_directories(${catkin_INCLUDE_DIRS})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(botPatternFormation src/botModelController.cpp src/FSM/FSM.cpp 
		src/FSM/StateImpulseSpeed.cpp src/FSM/StateCatchUp.cpp
		src/FSM/StateAlign.cpp src/FSM/StateHalt.cpp
		src/FSM/StateEvade.cpp src/FSM/StateCruise.cpp
		src/FSM/blobClass.h	)		 

target_link_libraries(botPatternFormation ${catkin_LIBRARIES})
add_dependencies(botPatternFormation vrep_common_generate_messages_cpp)


