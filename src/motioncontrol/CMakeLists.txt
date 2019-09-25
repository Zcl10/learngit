cmake_minimum_required(VERSION 3.6)
project(motioncontrol)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    ecucomm/include/AnalysisECU.h
    ecucomm/include/ECUDataOutPut.h
    ecucomm/src/ecu/AnalysisECU.cpp
    ecucomm/src/ecu/ECUDataOutPut.cpp
    ecucomm/src/util/boostudp/boostudp.cpp
    ecucomm/src/util/boostudp/boostudp.h
    ecucomm/src/util/playback/iv_data_playback.cpp
    ecucomm/src/util/playback/iv_data_playback.h
    ecucomm/src/util/utm/datum.cpp
    ecucomm/src/util/utm/datum.h
    ecucomm/src/util/utm/utm.cpp
    ecucomm/src/util/utm/utm.h
    ecucomm/src/util/xmlconf/tinyxml2.cpp
    ecucomm/src/util/xmlconf/tinyxml2.h
    ecucomm/src/util/xmlconf/xmlconf.cpp
    ecucomm/src/util/xmlconf/xmlconf.h
    ecucomm/src/util/common.h
    ecucomm/src/util/mytime.h
    ecucomm/src/util/NOTATION.hh
    ecucomm/src/util/ToXml.hh
    ecucomm/src/getecudata.cpp
    ecucomm/src/masternode.cpp
    ecucomm/src/sendecudata.cpp
    manual_gui/include/manual_gui_core.hpp
    manual_gui/src/manual_gui/manual_gui_core.cpp
    manual_gui/src/manual_gui/manual_gui_node.cpp)

add_executable(motioncontrol ${SOURCE_FILES})