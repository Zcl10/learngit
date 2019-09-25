cmake_minimum_required(VERSION 2.8.3)

project(ecucomm)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -std=c++11 ${CMAKE_CXX_FLAGS}")


## find ros packages and combine them with catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_driver_msgs
        std_msgs
		control_msgs
)

## find system depends libraries
#find_package(PCL REQUIRED)
#find_package(OpenCV REQUIRED)
# find_package(Protobuf REQUIRED)

## ros specific cmake command
catkin_package(
        INCLUDE_DIRS include
        # LIBRARIES ${PROJECT_NAME}_lib1
        CATKIN_DEPENDS ${catkin_deps}
        # DEPENDS OpenCV
)


#boost
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost COMPONENTS system filesystem REQUIRED)

#boost

include_directories(
        include
        # ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
		src
		${Boost_INCLUDE_DIRS}
)

## install

#install(FILES package.xml DESTINATION share/sensor_driver)
#
#install(DIRECTORY config DESTINATION share/sensor_driver/)
#
#install(DIRECTORY urdf DESTINATION share/sensor_driver/)

# set sources
file(GLOB_RECURSE ALL_UTIL_SRCS "src/util/*")
#

#############
## Build  ###
#############
set(ecu_SRC src/ecu/AnalysisECU.cpp src/ecu/ECUDataOutPut.cpp)
add_library(utils SHARED ${ALL_UTIL_SRCS} )
add_library(ivsensor SHARED ${ins_SRC} ${gps_SRC} ${ecu_SRC})



add_executable(getecudata
         src/getecudata.cpp
 	)
target_link_libraries(getecudata
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${LINKER_LIBS}
		utils
		ivsensor
		gflags
		glog
        )
        
add_executable(sendecudata
		src/sendecudata.cpp
		)
target_link_libraries(sendecudata
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${OpenCV_LIBS}
		${LINKER_LIBS}
		utils
		ivsensor
		gflags
		glog
		)
		
add_executable(ecuchannel
		src/masternode.cpp
		)
target_link_libraries(ecuchannel
		${catkin_LIBRARIES}
		${PCL_LIBRARIES}
		${OpenCV_LIBS}
		${LINKER_LIBS}
		utils
		ivsensor
		gflags
		glog
		${Boost_SYSTEM_LIBRARY}
		${Boost_LIBRARIES}
		)