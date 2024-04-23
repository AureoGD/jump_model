cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(jump_controller)
set (GZ_DISTRIBUTION "Harmonic")

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#============================================================================
# Find packages
#============================================================================
# If you get an error at this line, you need to install gz-cmake
find_package(gz-cmake3 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(gz-msgs10 REQUIRED)
# find_package(jump_msgs-msgs REQUIRED)

set(MSGS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../jump_msgs/build/proto) 
message(STATUS "################## ${MSGS_DIR}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
	${PYTHON_INCLUDE_DIRS}	
	/usr/local/include/
)

add_library(${PROJECT_NAME}
    SHARED
		src/jump_controller.cpp
		src/jump_low_level_controller.cpp
		src/jump_robot_model.cpp
		src/jump_rgc.cpp
		src/jump_interface.cpp
)

target_link_directories(${PROJECT_NAME}
    PUBLIC
	${MSGS_DIR}
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
		${GZ-SIM_LIBRARIES}
		${GZ-MATH_LIBRARIES}
		${GZ-COMMON_LIBRARIES}
		${GZ-SENSORS_LIBRARIES}
		osqp::osqp
		OsqpEigen::OsqpEigen
		jump_msgs-msgs
)	

add_subdirectory(python)

install(
	TARGETS             ${PROJECT_NAME}
    LIBRARY DESTINATION /usr/lib/gazebo/plugins
	COMPONENT           jump_controller
)