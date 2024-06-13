# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  actkin_balancer_msgs
  actkin_stabilizer_msgs
  rtm_data_tools
  contact_state_msgs
  eigen_rtm_conversions
  )
find_package(choreonoid REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  )

link_directories(
  ${openrtm_aist_LIBRARY_DIRS}
  )

add_library(ActKinBalancer SHARED
  ActKinBalancer.cpp
  ActKinBalancerService_impl.cpp
  State.cpp
  Goal.cpp
  FootStepGenerator.cpp
  )
target_link_libraries(ActKinBalancer
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  )
set_target_properties(ActKinBalancer PROPERTIES PREFIX "") # libActKinBalancer.so -> ActKinBalancer.so
add_dependencies(ActKinBalancer RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

add_executable(ActKinBalancerComp ActKinBalancerComp.cpp)
target_link_libraries(ActKinBalancerComp ActKinBalancer)
