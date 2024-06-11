#ifndef ActKinBalancer_H
#define ActKinBalancer_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/CorbaPort.h>
#include <rtm/idl/BasicDataType.hh>

#include <actkin_balancer_msgs/idl/ActKinBalancer.hh>

#include "ActKinBalancerService_impl.h"
#include "State.h"
#include "Goal.h"

class ActKinBalancer : public RTC::DataFlowComponentBase{
protected:
  class Ports {
  public:
    Ports();
    void onInitialize(ActKinBalancer* component);

    RTC::TimedDoubleSeq m_qRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn_;
    RTC::TimedDoubleSeq m_tauRef_;
    RTC::InPort<RTC::TimedDoubleSeq> m_tauRefIn_;
    RTC::TimedDoubleSeq m_qAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_qActIn_;
    RTC::TimedDoubleSeq m_dqAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn_;
    RTC::TimedDoubleSeq m_tauAct_;
    RTC::InPort<RTC::TimedDoubleSeq> m_tauActIn_;
    RTC::TimedDoubleSeq m_q_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_qOut_;
    RTC::TimedDoubleSeq m_tau_;
    RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut_;

    ActKinBalancerService_impl m_service0_;
    RTC::CorbaPort m_ActKinBalancerServicePort_;
  };
  Ports ports_;

public:
  ActKinBalancer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool startBalancer();
  bool stopBalancer();

  bool setActKinBalancerParam(const actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param);
  bool getActKinBalancerParam(actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param);

private:
};

extern "C"
{
  void ActKinBalancerInit(RTC::Manager* manager);
}

#endif // ActKinBalancer_H
