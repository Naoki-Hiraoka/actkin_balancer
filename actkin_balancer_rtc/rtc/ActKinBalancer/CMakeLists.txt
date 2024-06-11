add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  actkin_balancer_msgs
  )
include_directories(
  ${catkin_INCLUDE_DIRS}
  )
    
rtmbuild_add_library(ActKinBalancer SHARED
  ActKinBalancer.cpp
  ActKinBalancerService_impl.cpp
  )
target_link_libraries(ActKinBalancer
  ${catkin_LIBRARIES}
  )
set_target_properties(ActKinBalancer PROPERTIES PREFIX "") # libActKinBalancer.so -> ActKinBalancer.so                                                                           
add_dependencies(ActKinBalancer RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl                                                                                                        

rtmbuild_add_executable(ActKinBalancerComp ActKinBalancerComp.cpp)
target_link_libraries(ActKinBalancerComp ActKinBalancer)
