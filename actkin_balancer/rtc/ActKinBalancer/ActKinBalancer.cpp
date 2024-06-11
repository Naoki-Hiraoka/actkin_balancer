#include "ActKinBalancer.h"

ActKinBalancer::Ports::Ports() :
  m_qActIn_("qAct", m_qAct_),
  m_dqActIn_("dqAct", m_dqAct_),
  m_actBasePoseIn_("actBasePoseIn", m_actBasePose_),
  m_actBaseVelIn_("actBaseVelIn", m_actBaseVel_),
  m_actContactStateIn_("actContactStateIn", m_actContactState_),

  m_qOut_("q", m_q_),
  m_tauOut_("tauOut", m_tau_),
  m_ActKinBalancerServicePort_("ActKinBalancerService")
{
}

void ActKinBalancer::Ports::onInitialize(ActKinBalancer* component) {
  component->addInPort("qAct", this->m_qActIn_);
  component->addInPort("dqAct", this->m_dqActIn_);
  component->addInPort("actBasePoseIn", this->m_actBasePoseIn_);
  component->addInPort("actBaseVelIn", this->m_actBaseVelIn_);
  component->addInPort("actContactStateIn", this->m_actContactStateIn_);

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
    this->state_.onStartBalancer();
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
  bool qAct_updated = false;
  if(ports.m_qActIn_.isNew()) qAct_updated = true;
  while(ports.m_qActIn_.isNew()) ports.m_qActIn_.read();
  while(ports.m_dqActIn_.isNew()) ports.m_dqActIn_.read();
  while(ports.m_actBasePoseIn_.isNew()) ports.m_actBasePoseIn_.read();
  while(ports.m_actBaseVelIn_.isNew()) ports.m_actBaseVelIn_.read();
  state.updateRobotFromIdl(ports.m_qAct_, ports.m_dqAct_, ports.m_actBasePose_, ports.m_actBaseVel_, dt);

  while(ports.m_actContactStateIn_.isNew()) ports.m_actContactStateIn_.read();
  state.updateContactFromIdl(ports.m_actContactState_);

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
    ports.m_q_.tm = ports.m_qAct_.tm;
    ports.m_q_.data.length(0);
    for (int i = 0 ; i < 0; i++){
      ports.m_q_.data[i] = 0.0;
    }
    ports.m_qOut_.write();
  }

  {
    ports.m_tau_.tm = ports.m_qAct_.tm;
    ports.m_tau_.data.length(0);
    for (int i = 0 ; i < 0; i++){
      ports.m_tau_.data[i] = 0;
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
