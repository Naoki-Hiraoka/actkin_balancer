cmake_minimum_required(VERSION 3.0.2)
project(actkin_balancer_msgs)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

rtmbuild2_init(rtmbuild2) # ここにidlとmsgが依存する外部パッケージを書く

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES ActKinBalancerSkel ActKinBalancerStub
   DEPENDS openrtm_aist
)

###########
## Build ##
###########

rtmbuild2_genidl()

add_custom_target(genidl All DEPENDS RTMBUILD2_${PROJECT_NAME}_genrpc)
