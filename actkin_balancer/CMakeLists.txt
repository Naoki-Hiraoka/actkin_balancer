cmake_minimum_required(VERSION 3.0.2)
project(actkin_balancer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild2
  roscpp
  actkin_balancer_msgs
  )

rtmbuild2_init(rtmbuild2 actkin_balancer_msgs) # ここにidlとmsgが依存する外部パッケージを書く

catkin_package(
  LIBRARIES ActKinBalancerServiceSkel ActKinBalancerServiceStub
  CATKIN_DEPENDS rtmbuild2
  DEPENDS openrtm_aist
  )

# generate idl
rtmbuild2_genidl()

# generate bridge
rtmbuild2_genbridge()

add_subdirectory(rtc)
