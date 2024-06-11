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
  std::lock_guard<std::mutex> guard(this->mutex_);

  this->loop_++;
  std::string instance_name = std::string(this->m_profile.instance_name);
  double rate = this->get_context(ec_id)->get_rate();
  if(rate <= 0.0){
    std::cerr << "\x1b[31m[" << instance_name << "] " << " periodic rate is invalid " << rate << "\x1b[39m" << std::endl;
    return RTC::RTC_ERROR;
  }
  double dt = 1.0 / rate;

  // 外部からのサービスコールによって指令されたモード変更を反映する
  this->mode_.update();

  // startABC直後の一回のみ実行
  if(this->mode_.isSyncToABCInit()){
  }

  if(!ActKinBalancer::readInPortDataForState(this->ports_, instance_name, dt,
                                               this->state_)){
    return RTC::RTC_OK;  // qAct が届かなければ何もしない
  }

  if(this->mode_.isABCRunning()){
    ActKinBalancer::readInPortDataForGoal(this->ports_, instance_name, dt, this->state_,
                                            this->goal_);
  }

  ActKinBalancer::writeOutPortData(this->state_, this->mode_,
                                   this->ports_);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t ActKinBalancer::onActivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onActivated(" << ec_id << ")" << std::endl;
  this->mode_.reset();
  return RTC::RTC_OK;
}
RTC::ReturnCode_t ActKinBalancer::onDeactivated(RTC::UniqueId ec_id){
  std::lock_guard<std::mutex> guard(this->mutex_);
  std::cerr << "[" << m_profile.instance_name << "] "<< "onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}


// static function
bool ActKinBalancer::readInPortDataForState(ActKinBalancer::Ports& ports, const std::string& instance_name, const double& dt,
                                            actkin_balancer::State& state){


  if (ports.m_qRefIn_.isNew()){
    ports.m_qRefIn_.read();
  }

  if (ports.m_tauRefIn_.isNew()){
    ports.m_tauRefIn_.read();
  }

  if (ports.m_qActIn_.isNew()){
    ports.m_qActIn_.read();
  }

  if (ports.m_dqActIn_.isNew()){
    ports.m_dqActIn_.read();
  }

  if (ports.m_tauActIn_.isNew()){
    ports.m_tauActIn_.read();
  }

  return true;
}

// static function
bool ActKinBalancer::readInPortDataForGoal(ActKinBalancer::Ports& ports, const std::string& instance_name, const double& dt, const actkin_balancer::State& state,
                                             actkin_balancer::Goal& goal){
  return true;
}

// static function
bool ActKinBalancer::writeOutPortData(const actkin_balancer::State& state, const ActKinBalancer::ControlMode& mode,
                                        ActKinBalancer::Ports& ports){
  {
    ports.m_q_.tm = ports.m_qRef_.tm;
    ports.m_q_.data.length(ports.m_qRef_.data.length());
    for (int i = 0 ; i < ports.m_q_.data.length(); i++){
      ports.m_q_.data[i] = ports.m_qRef_.data[i];
    }
    ports.m_qOut_.write();
  }

  {
    ports.m_tau_.tm = ports.m_qRef_.tm;
    ports.m_tau_.data.length(ports.m_tauRef_.data.length());
    for (int i = 0 ; i < ports.m_q_.data.length(); i++){
      ports.m_tau_.data[i] = ports.m_tauRef_.data[i];
    }
    ports.m_tauOut_.write();
  }

  return true;
}


bool ActKinBalancer::startBalancer(){
  std::lock_guard<std::mutex> guard(this->mutex_);

  return true;
}
bool ActKinBalancer::stopBalancer(){
  std::lock_guard<std::mutex> guard(this->mutex_);

  return true;
}

bool ActKinBalancer::setActKinBalancerParam(const actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  return true;
}
bool ActKinBalancer::getActKinBalancerParam(actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
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
