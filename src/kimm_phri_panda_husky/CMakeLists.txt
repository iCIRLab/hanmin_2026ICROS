cmake_minimum_required(VERSION 2.8.3)
project(kimm_phri_panda_husky)

# --- hpp-fcl -> coal 호환 레이어 (새 환경에서도 옛 이름 유지)
if(NOT TARGET hpp-fcl::hpp-fcl)
  add_library(hpp-fcl::hpp-fcl INTERFACE IMPORTED)
  # coal 라이브러리에 포워딩 (robotpkg 최신은 libcoal.so 제공)
  find_library(COAL_LIB NAMES coal)
  if(COAL_LIB)
    # 필요 시 include 경로도 같이 넘겨줌
    set_target_properties(hpp-fcl::hpp-fcl PROPERTIES
      INTERFACE_LINK_LIBRARIES "${COAL_LIB}"
      INTERFACE_INCLUDE_DIRECTORIES "/opt/openrobots/include"
    )
  endif()
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}" )
SET(LIBRARY_NAME phri_hqp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mujoco_ros_msgs
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(kimm_hqp_controller REQUIRED)
find_library(YAML_CPP_LIBRARY yaml-cpp)
if(NOT YAML_CPP_LIBRARY)
  message(FATAL_ERROR "yaml-cpp library not found")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    mujoco_ros_msgs
    roscpp
    sensor_msgs
    std_msgs
)
###########
## Build ##
###########

## Whole-Body HQP library
SET(${LIBRARY_NAME}_HEADERS
  include/kimm_phri_panda_husky/panda_husky_hqp.h  
)
SET(${LIBRARY_NAME}_SOURCES
  src/panda_husky_hqp.cpp  
)
ADD_LIBRARY(${LIBRARY_NAME} SHARED ${${LIBRARY_NAME}_SOURCES} ${${LIBRARY_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${LIBRARY_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/openrobots/include
  ${catkin_INCLUDE_DIRS}
  ${kimm_hqp_controller_INCLUDE_DIRS}
)
TARGET_LINK_LIBRARIES(${LIBRARY_NAME} PUBLIC ${catkin_LIBRARIES} ${kimm_hqp_controller_LIBRARIES})


## Shared include dirs for the cooperative transport nodes
include_directories(
  include
  /opt/openrobots/include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Per-robot HQP controller for CM 기반 협업 운반.
add_executable(coop_husky_hqp_node src/coop_husky_hqp_node.cpp)
target_link_libraries(coop_husky_hqp_node
  ${catkin_LIBRARIES}
  ${LIBRARY_NAME}
  -lpthread
)
add_dependencies(coop_husky_hqp_node ${catkin_EXPORTED_TARGETS}
)

## Cooperative simulation runtime: attach/lift/CM readiness/monitor/command merge.
add_executable(coop_phri_simul src/coop_phri_simul.cpp)
target_link_libraries(coop_phri_simul
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARY}
)
add_dependencies(coop_phri_simul ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS ${LIBRARY_NAME} coop_husky_hqp_node coop_phri_simul
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
