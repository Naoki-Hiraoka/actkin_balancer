#include "State.h"
#include <iostream>
#include <rtm_data_tools/rtm_data_tools.h>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>
#include <cnoid/src/Body/InverseDynamics.h>
#include <cnoid/Jacobian>
#include <cnoid/SceneGraph>

namespace actkin_balancer{
  void EEParam::updateFromHull() {
    {
      // update WrenchC, wrenchld, wrenchud
      // 0 <  0  0  1  0  0  0 < 1e10
      // 0 <  1  0 mt  0  0  0 < 1e10
      // 0 < -1  0 mt  0  0  0 < 1e10
      // 0 <  0  1 mt  0  0  0 < 1e10
      // 0 <  0 -1 mt  0  0  0 < 1e10
      // 0 <  0  0  d r1 r2  0 < 1e10 ;; x hull.size()
      // 0 <  0  0 mr  0  0  1 < 1e10
      // 0 <  0  0 mr  0  0 -1 < 1e10
      int constraintDim = 7 + this->hull.size();
      this->wrenchC = Eigen::MatrixXd::Zero(constraintDim,6);
      this->wrenchld = Eigen::VectorXd::Zero(constraintDim);
      this->wrenchud = 1e10 * Eigen::VectorXd::Ones(constraintDim);
      int idx=0;
      this->wrenchC(idx,2) = 1.0; this->wrenchld[idx] = 50.0; idx++;
      this->wrenchC(idx,0) = 1.0; this->wrenchC(idx,2) = this->muTrans; idx++;
      this->wrenchC(idx,0) = -1.0; this->wrenchC(idx,2) = this->muTrans; idx++;
      this->wrenchC(idx,1) = 1.0; this->wrenchC(idx,2) = this->muTrans; idx++;
      this->wrenchC(idx,1) = -1.0; this->wrenchC(idx,2) = this->muTrans; idx++;
      for(int j=0;j<this->hull.size();j++){
        Eigen::Vector2d v1 = this->hull[j]; // EEF frame/origin
        Eigen::Vector2d v2 = this->hull[(j+1<this->hull.size())?j+1:0]; // EEF frame/origin
        if(v1.head<2>() == v2.head<2>()) continue;
        Eigen::Vector2d r = Eigen::Vector2d(v2[1]-v1[1],v1[0]-v2[0]).normalized();
        double d = r.dot(v1);
        this->wrenchC(idx,2) = d; this->wrenchC(idx,3) = -r[1]; this->wrenchC(idx,4) = r[0]; idx++;
      }
      this->wrenchC(idx,5) = 1.0; this->wrenchC(idx,2) = this->muRot; idx++;
      this->wrenchC(idx,5) = -1.0; this->wrenchC(idx,2) = this->muRot; idx++;
    }

    {
      // update region
      int regionDim = 1 + this->hull.size();
      this->region.C = Eigen::MatrixXd::Zero(regionDim,6);
      this->region.ld = -1e10 * Eigen::VectorXd::Zero(regionDim);
      this->region.ud = +1e10 * Eigen::VectorXd::Ones(regionDim);
      int idx=0;
      this->region.C(idx,2) = 1.0; this->region.ld[idx] = -this->regionMargin; this->region.ud[idx] = this->regionMargin;
      for(int j=0;j<this->hull.size();j++){
        Eigen::Vector2d v1 = this->hull[j]; // EEF frame/origin
        Eigen::Vector2d v2 = this->hull[(j+1<this->hull.size())?j+1:0]; // EEF frame/origin
        if(v1.head<2>() == v2.head<2>()) continue;
        Eigen::Vector2d r = Eigen::Vector2d(v2[1]-v1[1],v1[0]-v2[0]).normalized();
        double d = r.dot(v1);
        this->region.C(idx,0) = r[0]; this->region.C(idx,1) = r[1]; this->region.ud[idx] = d; idx++;
      }
    }

  }

  void EEParam::flipY(EEParam& param){
    std::string orgname = param.name;
    cnoid::LinkPtr orgLink = param.parentLink;
    param = *this;
    param.name = orgname;
    param.parentLink = orgLink;
    param.localPose.translation()[1] *= -1;
    param.localPose.linear() = param.localPose.linear().inverse(); // rx.ry=0と仮定している
    for(int i=0;i<param.hull.size();i++) param.hull[i][1] *= -1;
    for(int i=0;i<param.safeHull.size();i++) param.safeHull[i][1] *= -1;
    param.defaultTranslatePos[1] *= -1;
    for(int i=0;i<param.defaultStrideLimitationHull.size();i++) param.defaultStrideLimitationHull[i][1] *= -1;
    std::swap(param.strideLimitationMaxTheta, param.strideLimitationMinTheta);
    param.strideLimitationMaxTheta *= -1;
    param.strideLimitationMinTheta *= -1;
    for(int i=0;i<param.strideLimitationHull.size();i++) param.strideLimitationHull[i][1] *= -1;
    for(int i=0;i<param.wrenchC.rows();i++){
      param.wrenchC(i,1) *= -1;
      param.wrenchC(i,3) *= -1;
    }
    for(int i=0;i<param.region.C.rows();i++){
      param.region.C(i,1) *= -1;
    }
  }

  bool NominalEE::updateFromIdl(const State& state, const actkin_balancer::ActKinBalancerService::NominalEEIdl& idl){
    this->name = std::string(idl.name);
    if(state.nameLinkMap.find(std::string(idl.link)) == state.nameLinkMap.end()){
      std::cerr << idl.link << " not found" << std::endl;
      return false;
    }
    this->link = state.nameLinkMap.find(std::string(idl.link))->second;
    eigen_rtm_conversions::poseRTMToEigen(idl.localPose, this->localPose);
    if(state.nameLinkMap.find(std::string(idl.frameId)) == state.nameLinkMap.end()){
      std::cerr << idl.frameId << " not found" << std::endl;
      return false;
    }
    this->frameLink = state.nameLinkMap.find(std::string(idl.frameId))->second;
    eigen_rtm_conversions::poseRTMToEigen(idl.framePose, this->framePose);
    this->time = std::max(0.0, idl.time);
    eigen_rtm_conversions::poseRTMToEigen(idl.pose, this->pose);
    for(int i=0;i<6;i++) this->freeAxis[i] = idl.freeAxis[i];
    this->priority = idl.priority;

    return true;
  }

  void NominalEE::convertToIdl(const State& state, actkin_balancer::ActKinBalancerService::NominalEEIdl& idl) {
    idl.name = this->name.c_str();
    idl.link = state.linkNameMap.find(this->link)->second.c_str();
    eigen_rtm_conversions::poseEigenToRTM(this->localPose,idl.localPose);
    idl.frameId = state.linkNameMap.find(this->frameLink)->second.c_str();
    eigen_rtm_conversions::poseEigenToRTM(this->framePose,idl.framePose);
    idl.time = this->time;
    eigen_rtm_conversions::poseEigenToRTM(this->pose,idl.pose);
    for(int i=0;i<6;i++) idl.freeAxis[i] = this->freeAxis[i];
    idl.priority = this->priority;
  }

  bool NominalInfo::updateFromIdl(const State& state, const actkin_balancer::ActKinBalancerService::NominalIdl& idl) {
    this->nominalqTime = std::max(0.0, idl.nominalqTime);
    eigen_rtm_conversions::vectorRTMToEigen(idl.nominalq, this->nominalq);
    this->nominalEE.clear();
    for(int i=0;i<idl.nominalEE.length();i++){
      NominalEE nominalEE_;
      if(nominalEE_.updateFromIdl(state, idl.nominalEE[i])){
        this->nominalEE.push_back(nominalEE_);
      }
    }
    this->nominalZ = std::max(0.01, idl.nominalZ);
    return true;
  }
  void NominalInfo::convertToIdl(const State& state, actkin_balancer::ActKinBalancerService::NominalIdl& idl) {
    idl.nominalqTime = this->nominalqTime;
    eigen_rtm_conversions::vectorEigenToRTM(this->nominalq, idl.nominalq);
    idl.nominalEE.length(this->nominalEE.size());
    for(int i=0;i<this->nominalEE.size();i++){
      this->nominalEE[i].convertToIdl(state, idl.nominalEE[i]);
    }
    idl.nominalZ = this->nominalZ;
  }

  void State::init(const cnoid::BodyPtr& robot_){

    this->robot = robot_;

    this->nameLinkMap[std::string("")] = nullptr; //world
    this->linkNameMap[nullptr] = std::string(""); //world
    for(int l=0;l<this->robot->numLinks() ; l++){
      cnoid::SgGroup* shape = this->robot->link(l)->shape();
      if(shape && shape->numChildObjects() > 0 && shape->child(0)->name().size()!=0){
        this->nameLinkMap[shape->child(0)->name()] = this->robot->link(l);
        this->linkNameMap[this->robot->link(l)] = shape->child(0)->name();
      }
    }

    this->ee.resize(NUM_LEGS);
    this->ee[RLEG].updateFromHull();
    this->ee[RLEG].flipY(this->ee[LLEG]);

    return;
  };

  bool State::onStartBalancer(){
    return true;
  }

  void State::updateRobotFromIdl(const RTC::TimedDoubleSeq& m_qAct, const RTC::TimedDoubleSeq& m_dqAct, const RTC::TimedPose3D& m_actBasePose, const RTC::TimedVelocity3D& m_actBaseVel, double dt) {
    if(m_qAct.data.length() == this->robot->numJoints()){
      if(rtm_data_tools::isAllFinite(m_qAct.data)){
        for(int i=0;i<m_qAct.data.length();i++){
          this->robot->joint(i)->q() = m_qAct.data[i];
        }
      }else{
        std::cerr << "m_qAct is not finite!" << std::endl;
      }
    }
    if(m_dqAct.data.length() == this->robot->numJoints()){
      if(rtm_data_tools::isAllFinite(m_dqAct.data)){
        for(int i=0;i<m_dqAct.data.length();i++){
          this->robot->joint(i)->dq() = m_dqAct.data[i];
        }
      }else{
        std::cerr << "m_dqAct is not finite!" << std::endl;
      }
    }
    if(rtm_data_tools::isAllFinite(m_actBasePose.data)){
      eigen_rtm_conversions::poseRTMToEigen(m_actBasePose.data,this->robot->rootLink()->T());
    }else{
      std::cerr << "m_actBasePose is not finite!" << std::endl;
    }
    if(rtm_data_tools::isAllFinite(m_actBaseVel.data)){
      eigen_rtm_conversions::velocityRTMToEigen(m_actBaseVel.data,this->robot->rootLink()->v(),this->robot->rootLink()->w());
    }else{
      std::cerr << "m_actBaseVel is not finite!" << std::endl;
    }

    this->robot->rootLink()->dv().setZero();
    this->robot->rootLink()->dw().setZero();
    for(int i=0;i<this->robot->numJoints();i++) {
      this->robot->joint(i)->ddq() = 0.0;
    }
    for(int i=0;i<this->robot->numLinks();i++) {
      this->robot->link(i)->F_ext().setZero();
    }
    this->robot->calcForwardKinematics(true,true);
    this->robot->calcCenterOfMass();
    cnoid::Vector6 F_o = cnoid::calcInverseDynamics(this->robot->rootLink()); // world frame origin
    this->robot->rootLink()->F_ext().head<3>() = F_o.head<3>(); // rootLink origin
    this->robot->rootLink()->F_ext().tail<3>() = F_o.tail<3>() + (-this->robot->rootLink()->p()).cross(F_o.head<3>()); // rootLink origin

    {
      Eigen::MatrixXd CMJ;
      cnoid::calcCMJacobian(this->robot,nullptr,CMJ); // [joint root]の順
      cnoid::VectorX dq(this->robot->numJoints()+6);
      for(int i=0;i<this->robot->numJoints();i++) dq[i] = this->robot->joint(i)->dq();
      dq.segment<3>(this->robot->numJoints()) = this->robot->rootLink()->v();
      dq.tail<3>() = this->robot->rootLink()->w();
      //this->cogVel.passFilter(CMJ * dq, dt);
      this->cogVel = CMJ * dq;
    }
  }

  void State::updateContactFromIdl(const contact_state_msgs::TimedContactSeq& m_actContactState){
    // update contacts
    this->contacts.resize(m_actContactState.data.length());
    int numContact= 0;
    for(int i=0;i<m_actContactState.data.length();i++){
      if(!this->contacts[numContact]) this->contacts[numContact] = std::make_shared<Contact>();
      if(this->nameLinkMap.find(std::string(m_actContactState.data[i].link1)) == this->nameLinkMap.end()){
        std::cerr << __FUNCTION__ << m_actContactState.data[i].link1 << " not found" << std::endl;
        continue;
      }
      this->contacts[numContact]->link1 = this->nameLinkMap[std::string(m_actContactState.data[i].link1)];
      if(!rtm_data_tools::isAllFinite(m_actContactState.data[i].local_pose)){
        std::cerr << __FUNCTION__ << "local_pose not finite" << std::endl;
        continue;
      }
      eigen_rtm_conversions::poseRTMToEigen(m_actContactState.data[i].local_pose, this->contacts[numContact]->localPose1);
      if(this->nameLinkMap.find(std::string(m_actContactState.data[i].link2)) == this->nameLinkMap.end()){
        std::cerr << __FUNCTION__ << m_actContactState.data[i].link2 << " not found" << std::endl;
        continue;
      }
      this->contacts[numContact]->link2 = this->nameLinkMap[std::string(m_actContactState.data[i].link2)];
      this->contacts[numContact]->freeX = m_actContactState.data[i].free_x;
      this->contacts[numContact]->freeY = m_actContactState.data[i].free_y;
      numContact++;
    }
    this->contacts.resize(numContact);

    // update actContacts
    for(int LEG=0;LEG<NUM_LEGS;LEG++){
      cnoid::Isometry3 poseInv = this->ee[LEG].parentLink->T() * this->ee[LEG].localPose;
      this->actContact[LEG] = false;
      for(int i=0;i<this->contacts.size();i++){
        if( ((this->contacts[i]->link1 == this->ee[LEG].parentLink) && (this->contacts[i]->link2 == nullptr)) ||
            ((this->contacts[i]->link1 == nullptr) && (this->contacts[i]->link2 == this->ee[LEG].parentLink)) ) {
          cnoid::Vector3 p = (this->contacts[i]->link1 ? this->contacts[i]->link1->T() * this->contacts[i]->localPose1.translation() : this->contacts[i]->localPose1.translation()); // world frame
          cnoid::Vector3 value = this->ee[LEG].region.C * (poseInv * p);
          // TODO 法線方向のチェック.
          if(// region
             ((value - this->ee[LEG].region.ld).array() >= 0.0).all() &&
             ((this->ee[LEG].region.ud - value).array() >= 0.0).all() &&
             // 重心より低い
             p[2] < this->robot->centerOfMass()[2]
             ){
            this->actContact[LEG] = true;
            break;
          }
        }
      }

    }
  }

};
