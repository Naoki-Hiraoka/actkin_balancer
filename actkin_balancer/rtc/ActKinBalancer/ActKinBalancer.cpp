#include "ActKinBalancer.h"
#include <cnoid/BodyLoader>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>

ActKinBalancer::Ports::Ports() :
  m_qActIn_("qAct", m_qAct_),
  m_dqActIn_("dqAct", m_dqAct_),
  m_actBasePoseIn_("actBasePoseIn", m_actBasePose_),
  m_actBaseVelIn_("actBaseVelIn", m_actBaseVel_),
  m_actContactStateIn_("actContactStateIn", m_actContactState_),

  m_refStateIn_("refStateIn",m_refState_),

  m_outStateOut_("outStateOut",m_outState_),
  m_ActKinBalancerServicePort_("ActKinBalancerService")
{
}

void ActKinBalancer::Ports::onInitialize(ActKinBalancer* component) {
  component->addInPort("qAct", this->m_qActIn_);
  component->addInPort("dqAct", this->m_dqActIn_);
  component->addInPort("actBasePoseIn", this->m_actBasePoseIn_);
  component->addInPort("actBaseVelIn", this->m_actBaseVelIn_);
  component->addInPort("actContactStateIn", this->m_actContactStateIn_);

  component->addInPort("refStateIn", this->m_refStateIn_);

  component->addOutPort("outStateOut", this->m_outStateOut_);

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


  {
    // load robot model
    cnoid::BodyPtr robot;
    {
      cnoid::BodyLoader bodyLoader;
      std::string fileName; this->getProperty("model", fileName);
      if (fileName.find("file://") == 0) fileName.erase(0, strlen("file://"));
      robot = bodyLoader.load(fileName);
      if(!robot){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "failed to load model[" << fileName << "]" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      if(!robot->rootLink()->isFreeJoint()){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << "rootLink is not FreeJoint [" << fileName << "]" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
    }

    this->state_.init(robot);
  }

  {
    // load end_effector
    std::string endEffectors; this->getProperty("end_effectors", endEffectors);
    std::stringstream ss_endEffectors(endEffectors);
    std::string buf;
    int leg = 0;
    while(leg<2 && std::getline(ss_endEffectors, buf, ',')){
      std::string name;
      std::string parentLink;
      cnoid::Vector3 localp;
      cnoid::Vector3 localaxis;
      double localangle;

      //   name, parentLink(VRML joint name), (not used), x, y, z, theta, ax, ay, az
      name = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; parentLink = buf;
      if(!std::getline(ss_endEffectors, buf, ',')) break; // not used
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localp[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[0] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[1] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localaxis[2] = std::stod(buf);
      if(!std::getline(ss_endEffectors, buf, ',')) break; localangle = std::stod(buf);

      // check validity
      name.erase(std::remove(name.begin(), name.end(), ' '), name.end()); // remove whitespace
      parentLink.erase(std::remove(parentLink.begin(), parentLink.end(), ' '), parentLink.end()); // remove whitespace
      if(!this->state_.robot->link(parentLink)){
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " link [" << parentLink << "]" << " is not found for " << name << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      cnoid::Matrix3 localR;
      if(localaxis.norm() == 0) localR = cnoid::Matrix3::Identity();
      else localR = Eigen::AngleAxisd(localangle, localaxis.normalized()).toRotationMatrix();
      cnoid::Isometry3 localT;
      localT.translation() = localp;
      localT.linear() = localR;

      if(leg==actkin_balancer::RLEG && name != "rleg") {
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " eename " << name << " != \"rleg\"" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      if(leg==actkin_balancer::LLEG && name != "lleg") {
        std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " eename " << name << " != \"lleg\"" << "\x1b[39m" << std::endl;
        return RTC::RTC_ERROR;
      }
      this->state_.ee[leg].name = name;
      this->state_.ee[leg].parentLink = this->state_.robot->link(parentLink);
      this->state_.ee[leg].localPose = localT;

      leg++;
    }
    if(leg!=actkin_balancer::NUM_LEGS) {
      std::cerr << "\x1b[31m[" << this->m_profile.instance_name << "] " << " ee < 2" << "\x1b[39m" << std::endl;
      return RTC::RTC_ERROR;
    }
  }


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
    this->footStepGenerator_.onStartBalancer(this->state_);
  }

  if(this->mode_.isABCRunning()){
    if(!ActKinBalancer::readInPortDataForState(this->ports_, instance_name, dt,
                                               this->state_)){
      return RTC::RTC_OK;  // qAct が届かなければ何もしない
    }
    ActKinBalancer::readInPortDataForGoal(this->ports_, instance_name, dt, this->state_,
                                            this->goal_);
    this->footStepGenerator_.calcFootSteps(this->state_, this->goal_, instance_name, dt,
                                           this->output_);
    ActKinBalancer::writeOutPortData(this->state_, this->output_, this->mode_,
                                     this->ports_);
  }

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

  if(ports.m_refStateIn_.isNew() || ports.m_refStateUpdatedByService_){
    while(ports.m_refStateIn_.isNew()) ports.m_refStateIn_.read();
    ports.m_refStateUpdatedByService_ = false;
    goal.updateFromIdl(state, ports.m_refState_);
  }

  return true;
}

// static function
bool ActKinBalancer::writeOutPortData(const actkin_balancer::State& state, const actkin_balancer::Output& output, const ActKinBalancer::ControlMode& mode,
                                      ActKinBalancer::Ports& ports){
  if(mode.isABCRunning()){
    output.convertToIdl(state, ports.m_outState_);
    ports.m_outStateOut_.write();
  }
  return true;
}


bool ActKinBalancer::startBalancer(){
  if(this->mode_.setNextTransition(ControlMode::START_ABC)){
    std::cerr << "[" << m_profile.instance_name << "] start ABC" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_ABC) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] already started" << std::endl;
    return false;
  }

  return true;
}
bool ActKinBalancer::stopBalancer(){
  if(this->mode_.setNextTransition(ControlMode::STOP_ABC)){
    std::cerr << "[" << m_profile.instance_name << "] stop ABC" << std::endl;
    while (this->mode_.now() != ControlMode::MODE_IDLE) usleep(1000);
    usleep(1000);
    return true;
  }else{
    std::cerr << "[" << this->m_profile.instance_name << "] already stopped" << std::endl;
    return false;
  }

  return true;
}

bool ActKinBalancer::setActKinBalancerParam(const actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);
  if(this->mode_.isABCRunning()) return true;

  if(this->state_.nameLinkMap.find(std::string(i_param.rlegLink)) == this->state_.nameLinkMap.end() ||
     this->state_.nameLinkMap[std::string(i_param.rlegLink)] == nullptr
     ){
    std::cerr << "[" << this->m_profile.instance_name << "] " << i_param.rlegLink << " not found" << std::endl;
  }else{
    this->state_.ee[actkin_balancer::RLEG].parentLink = this->state_.nameLinkMap[std::string(i_param.rlegLink)];
  }
  if(this->state_.nameLinkMap.find(std::string(i_param.llegLink)) == this->state_.nameLinkMap.end() ||
     this->state_.nameLinkMap[std::string(i_param.llegLink)] == nullptr
     ){
    std::cerr << "[" << this->m_profile.instance_name << "] " << i_param.llegLink << " not found" << std::endl;
  }else{
    this->state_.ee[actkin_balancer::LLEG].parentLink = this->state_.nameLinkMap[std::string(i_param.llegLink)];
  }
  this->state_.nominal.updateFromIdl(this->state_, i_param.nominal);
  eigen_rtm_conversions::poseRTMToEigen(i_param.localPose, this->state_.ee[actkin_balancer::RLEG].localPose);

  this->state_.ee[actkin_balancer::RLEG].updateFromHull();
  this->state_.ee[actkin_balancer::RLEG].flipY(this->state_.ee[actkin_balancer::LLEG]);

  return true;
}
bool ActKinBalancer::getActKinBalancerParam(actkin_balancer::ActKinBalancerService::ActKinBalancerParam& i_param){
  std::lock_guard<std::mutex> guard(this->mutex_);

  i_param.rlegLink = this->state_.linkNameMap[this->state_.ee[actkin_balancer::RLEG].parentLink].c_str();
  i_param.llegLink = this->state_.linkNameMap[this->state_.ee[actkin_balancer::LLEG].parentLink].c_str();
  this->state_.nominal.convertToIdl(this->state_, i_param.nominal);
  eigen_rtm_conversions::poseEigenToRTM(this->state_.ee[actkin_balancer::RLEG].localPose, i_param.localPose);

  return true;
}

bool ActKinBalancer::setRefState(const actkin_balancer_msgs::RefStateIdl& i_param) {
  std::lock_guard<std::mutex> guard(this->mutex_);
  this->ports_.m_refState_ = i_param;
  this->ports_.m_refStateUpdatedByService_ = true;
  return true;
}

bool ActKinBalancer::getProperty(const std::string& key, std::string& ret) {
  if (this->getProperties().hasKey(key.c_str())) {
    ret = std::string(this->getProperties()[key.c_str()]);
  } else if (this->m_pManager->getConfig().hasKey(key.c_str())) { // 引数 -o で与えたプロパティを捕捉
    ret = std::string(this->m_pManager->getConfig()[key.c_str()]);
  } else {
    return false;
  }
  std::cerr << "[" << this->m_profile.instance_name << "] " << key << ": " << ret <<std::endl;
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
