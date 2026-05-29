cmake_minimum_required(VERSION 2.8.3)
project(xf_mic_asr_offline)
add_compile_options(-std=c++11)
set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
set (CMAKE_VERBOSE_MAKEFILE ON)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

#link_directories(
#  lib/arm64
#)
link_directories(
  /home/ubuntu/share/tmp/xf_mic_asr_offline/lib/arm64
  /home/ubuntu/jetarm/src/xf_mic_asr_offline/lib/arm64
  /usr/lib
)


catkin_python_setup()

add_message_files(
  FILES
  BuzzerState.msg
)

add_service_files(
  FILES
  GetOfflineResult.srv
  SetString.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(voice_control src/voice_control.cpp)
add_dependencies(
  voice_control
  ${xf_mic_package_EXPORTED_TARGETS}
  xf_mic_asr_offline_gencpp
)
target_link_libraries(voice_control ${catkin_LIBRARIES} offline_record_lib  rt dl pthread stdc++ )
find_library(HID_LIB NAMES hid_lib PATHS /usr/lib/)  
if(NOT HID_LIB)  
    message(FATAL_ERROR "hid_lib library not found")  
endif()  
#target_link_libraries(voice_control ${HID_LIB})


#target_link_libraries(voice_control ${HID_LIB}  ${catkin_LIBRARIES} offline_record_lib hid_lib rt dl pthread stdc++ )
