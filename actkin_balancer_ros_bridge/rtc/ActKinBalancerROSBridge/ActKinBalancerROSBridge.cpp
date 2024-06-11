#include "ActKinBalancerROSBridge.h"

ActKinBalancerROSBridge::ActKinBalancerROSBridge(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
{
}

RTC::ReturnCode_t ActKinBalancerROSBridge::onInitialize(){
  ros::NodeHandle pnh("~");
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t ActKinBalancerROSBridge::onExecute(RTC::UniqueId ec_id){
  ros::spinOnce();

  return RTC::RTC_OK;
}


static const char* ActKinBalancerROSBridge_spec[] = {
  "implementation_id", "ActKinBalancerROSBridge",
  "type_name",         "ActKinBalancerROSBridge",
  "description",       "ActKinBalancerROSBridge component",
  "version",           "0.0",
  "vendor",            "Takuma-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};

extern "C"{
    void ActKinBalancerROSBridgeInit(RTC::Manager* manager) {
        RTC::Properties profile(ActKinBalancerROSBridge_spec);
        manager->registerFactory(profile, RTC::Create<ActKinBalancerROSBridge>, RTC::Delete<ActKinBalancerROSBridge>);
    }
};
