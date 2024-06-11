# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actkin_balancer_msgs)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS})

add_library(ActKinBalancerROSBridge SHARED ActKinBalancerROSBridge.cpp)
target_link_libraries(ActKinBalancerROSBridge
  ${catkin_LIBRARIES}
  ${openrtm_aist_LIBRARIES})
set_target_properties(ActKinBalancerROSBridge PROPERTIES PREFIX "")

add_executable(ActKinBalancerROSBridgeComp ActKinBalancerROSBridgeComp.cpp)
target_link_libraries(ActKinBalancerROSBridgeComp ActKinBalancerROSBridge)

install(TARGETS ActKinBalancerROSBridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS ActKinBalancerROSBridgeComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
