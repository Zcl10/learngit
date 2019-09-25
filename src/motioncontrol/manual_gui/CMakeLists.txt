cmake_minimum_required(VERSION 2.8.3)
project(manual_gui)
find_package(catkin REQUIRED COMPONENTS
		roscpp
		std_msgs
		rosparam_handler
		dynamic_reconfigure
		control_msgs
		)

# set compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" Cpp14CompilerFlag)
if (${Cpp14CompilerFlag})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC" AND NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS "17")
	#no additional flag is required
else()
	message(FATAL_ERROR "Compiler does not have c++14 support. Use at least g++4.9 or Visual Studio 2013 and newer.")
endif()

# Generate parameter files
generate_ros_parameter_files(cfg/Demo.params)

# Create package
catkin_package(
		CATKIN_DEPENDS ${catkin_deps}
)
include_directories(
		include
		${catkin_INCLUDE_DIRS}
)

# add a ros node executable file
add_executable(${PROJECT_NAME}
		src/manual_gui/manual_gui_core.cpp
		src/manual_gui/manual_gui_node.cpp
		)

add_dependencies(${PROJECT_NAME}
		${catkin_EXPORTED_TARGETS}
		${PROJECT_NAME}_gencfg
		)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
		ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
		RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
		)

