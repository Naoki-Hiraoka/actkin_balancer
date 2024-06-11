#include "ActKinBalancer.h"

ActKinBalancer::Ports::Ports() :
  m_qRefIn_("qRefIn", m_qRef_),
  m_tauRefIn_("tauRefIn", m_tauRef_),
  m_qActIn_("qActIn", m_qAct_),
  m_dqActIn_("dqActIn", m_dqAct_),
  m_tauActIn_("tauActIn", m_tauAct_),
  m_qOut_("q", m_q_),
  m_tauOut_("tauOut", m_tau_),
  m_ActKinBalancerServicePort_("ActKinBalancerService")
{
}

void ActKinBalancer::Ports::onInitialize(ActKinBalancer* component) {
  component->addInPort("qRefIn", this->m_qRefIn_);
  component->addInPort("tauRefIn", this->m_tauRefIn_);
  component->addInPort("qActIn", this->m_qActIn_);
  component->addInPort("dqActIn", this->m_dqActIn_);
  component->addInPort("tauActIn", this->m_tauActIn_);
  component->addOutPort("q", this->m_qOut_);
  component->addOutPort("tauOut", this->m_tauOut_);

  this->m_ActKinBalancerServicePort_.registerProvider("service0", "ActKinBalancerService", this->m_service0_);
  component->addPort(this->m_ActKinBalancerServicePort_);

}

ActKinBalancer::ActKinBalancer(RTC::Manager* manager):
  RTC::DataFlowComponentBase(manager)
{
  this->ports_.m_service0_.setComp(this);
}

RTC::ReturnCode_t ActKinBalancer::onInitialize(){
  this->ports_.onInitialize(this);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t ActKinBalancer::onExecute(RTC::UniqueId ec_id){

  if (this->ports_.m_qRefIn_.isNew()){
    this->ports_.m_qRefIn_.read();
  }

  if (this->ports_.m_tauRefIn_.isNew()){
    this->ports_.m_tauRefIn_.read();
  }

  if (this->ports_.m_qActIn_.isNew()){
    this->ports_.m_qActIn_.read();
  }

  if (this->ports_.m_dqActIn_.isNew()){
    this->ports_.m_dqActIn_.read();
  }

  if (this->ports_.m_tauActIn_.isNew()){
    this->ports_.m_tauActIn_.read();
  }

  {
    this->ports_.m_q_.tm = this->ports_.m_qRef_.tm;
    this->ports_.m_q_.data.length(this->ports_.m_qRef_.data.length());
    for (int i = 0 ; i < this->ports_.m_q_.data.length(); i++){
      this->ports_.m_q_.data[i] = this->ports_.m_qRef_.data[i];
    }
    this->ports_.m_qOut_.write();
  }

  {
    this->ports_.m_tau_.tm = this->ports_.m_qRef_.tm;
    this->ports_.m_tau_.data.length(this->ports_.m_tauRef_.data.length());
    for (int i = 0 ; i < this->ports_.m_q_.data.length(); i++){
      this->ports_.m_tau_.data[i] = this->ports_.m_tauRef_.data[i];
    }
    this->ports_.m_tauOut_.write();
  }

  return RTC::RTC_OK;
}

bool ActKinBalancer::startBalancer(){
  return true;
}
bool ActKinBalancer::stopBalancer(){
  return true;
}

bool ActKinBalancer::setActKinBalancerParam(const actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param){
  return true;
}
bool ActKinBalancer::getActKinBalancerParam(actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param){
  return true;
}


static const char* ActKinBalancer_spec[] = {
  "implementation_id", "ActKinBalancer",
  "type_name",         "ActKinBalancer",
  "description",       "ActKinBalancer component",
  "version",           "0.0",
  "vendor",            "Naoki-Hiraoka",
  "category",          "example",
  "activity_type",     "DataFlowComponent",
  "max_instance",      "10",
  "language",          "C++",
  "lang_type",         "compile",
  ""
};


extern "C"{
  void ActKinBalancerInit(RTC::Manager* manager) {
    RTC::Properties profile(ActKinBalancer_spec);
    manager->registerFactory(profile, RTC::Create<ActKinBalancer>, RTC::Delete<ActKinBalancer>);
  }
};
