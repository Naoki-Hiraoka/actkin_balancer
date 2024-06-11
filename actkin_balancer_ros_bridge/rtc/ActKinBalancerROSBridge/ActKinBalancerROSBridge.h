#ifndef ActKinBalancerROSBridge_H
#define ActKinBalancerROSBridge_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>

#include <actkin_balancer_msgs/idl/ActKinBalancer.hh>

#include <ros/ros.h>

class ActKinBalancerROSBridge : public RTC::DataFlowComponentBase{
protected:
  ros::NodeHandle nh;

public:
  ActKinBalancerROSBridge(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
};

extern "C"
{
  void ActKinBalancerROSBridgeInit(RTC::Manager* manager);
};

#endif // ActKinBalancerROSBridge_H
