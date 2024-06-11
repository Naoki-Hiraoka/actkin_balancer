#ifndef ActKinBalancer_H
#define ActKinBalancer_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataOutPort.h>
#include <rtm/DataInPort.h>
#include <rtm/CorbaPort.h>
#include <rtm/idl/BasicDataType.hh>

#include <mutex>

#include <actkin_balancer_msgs/idl/ActKinBalancer.hh>

#include "ActKinBalancerService_impl.h"
#include "State.h"
#include "Goal.h"

class ActKinBalancer : public RTC::DataFlowComponentBase{
public:
  ActKinBalancer(RTC::Manager* manager);
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  bool startBalancer();
  bool stopBalancer();

  bool setActKinBalancerParam(const actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param);
  bool getActKinBalancerParam(actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param);

protected:
  std::mutex mutex_;

  unsigned long long loop_ = 0;
  int debugLevel_ = 0;

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

  class ControlMode{
  public:
    /*
      MODE_IDLE -> startStabilizer() -> MODE_ST
      MODE_ST -> stopStabilizer() -> MODE_IDLE

      MODE_IDLEの場合はstateのみ受け取り他は何もしない.
      MODE_STの場合はgoalも受け取り、torqueを出力する.
      切り替えの連続性を担保するにはこのRTCではない.
     */
    enum Mode_enum{ MODE_IDLE, MODE_ABC};
    enum Transition_enum{ START_ABC, STOP_ABC};
  private:
    Mode_enum current, previous, next;
  public:
    ControlMode(){ reset(); }
    void reset(){ current = previous = next = MODE_IDLE; }
    bool setNextTransition(const Transition_enum request){
      switch(request){
      case START_ABC:
        if(current == MODE_IDLE){ next = MODE_ABC; return true; }else{ return false; }
      case STOP_ABC:
        if(current == MODE_ABC){ next = MODE_IDLE; return true; }else{ return false; }
      default:
        return false;
      }
    }
    void update(){
      previous = current; current = next;
    }
    Mode_enum now() const{ return current; }
    Mode_enum pre() const{ return previous; }
    bool isSyncToABCInit() const{ return (current != previous) && (current==MODE_ABC);}
    bool isSyncToIdleInit() const{ return (current != previous) && (current==MODE_IDLE);}
    bool isABCRunning() const{ return (current==MODE_ABC) ;}
  };
  ControlMode mode_;

  actkin_balancer::State state_;
  actkin_balancer::Goal goal_;
protected:
  static bool readInPortDataForState(ActKinBalancer::Ports& ports, const std::string& instance_name, const double& dt,
                                     actkin_balancer::State& state);
  static bool readInPortDataForGoal(ActKinBalancer::Ports& ports, const std::string& instance_name, const double& dt, const actkin_balancer::State& state,
                                    actkin_balancer::Goal& goal);
  static bool writeOutPortData(const actkin_balancer::State& state, const ActKinBalancer::ControlMode& mode,
                               ActKinBalancer::Ports& ports);
};

extern "C"
{
  void ActKinBalancerInit(RTC::Manager* manager);
}

#endif // ActKinBalancer_H
