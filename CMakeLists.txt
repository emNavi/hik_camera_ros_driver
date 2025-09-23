cmake_minimum_required(VERSION 3.0.2)
project(hik_camera_driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -march=native -O3 -pthread")
SET(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  config/camera.cfg
)

catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(MVS_LIBRARIES libMvCameraControl.so
                  libMVGigEVisionSDK.so
                  libMVRender.so
                  libMvUsb3vTL.so
                  libMediaProcess.so
                  )

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    set(ARCH "aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(ARCH "x86_64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

if(ARCH STREQUAL "aarch64")
    LINK_DIRECTORIES(
        /opt/MVS/lib/aarch64
      )
elseif(ARCH STREQUAL "x86_64")
    LINK_DIRECTORIES(
        /opt/MVS/lib/64
      )
endif()

add_executable(camera_driver src/hik_driver.cpp
                             src/camera.cpp)

add_dependencies(camera_driver ${PROJECT_NAME}_gencfg)

target_link_libraries(camera_driver 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${MVS_LIBRARIES}
  )

install(TARGETS camera_driver
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

install(DIRECTORY libs/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/libs)
