#include "State.h"
#include <iostream>
#include <rtm_data_tools/rtm_data_tools.h>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>
#include <cnoid/src/Body/InverseDynamics.h>
#include <cnoid/Jacobian>
#include <cnoid/SceneGraph>

namespace actkin_balancer{
  void State::init(const cnoid::BodyPtr& robot_){

    this->robot = robot_;

    this->linkNameMap[std::string("")] = nullptr; //world
    for(int l=0;l<this->robot->numLinks() ; l++){
      cnoid::SgGroup* shape = this->robot->link(l)->shape();
      if(shape && shape->numChildObjects() > 0 && shape->child(0)->name().size()!=0){
        this->linkNameMap[shape->child(0)->name()] = this->robot->link(l);
      }
    }

    return;
  };

  void State::onStartBalancer(){
    return;
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
    this->contacts.resize(m_actContactState.data.length());
    int numContact= 0;
    for(int i=0;i<m_actContactState.data.length();i++){
      if(!this->contacts[numContact]) this->contacts[numContact] = std::make_shared<Contact>();
      if(this->linkNameMap.find(std::string(m_actContactState.data[i].link1)) == this->linkNameMap.end()){
        std::cerr << __FUNCTION__ << m_actContactState.data[i].link1 << " not found" << std::endl;
        continue;
      }
      this->contacts[numContact]->link1 = this->linkNameMap[std::string(m_actContactState.data[i].link1)];
      if(!rtm_data_tools::isAllFinite(m_actContactState.data[i].local_pose)){
        std::cerr << __FUNCTION__ << "local_pose not finite" << std::endl;
        continue;
      }
      eigen_rtm_conversions::poseRTMToEigen(m_actContactState.data[i].local_pose, this->contacts[numContact]->localPose1);
      if(this->linkNameMap.find(std::string(m_actContactState.data[i].link2)) == this->linkNameMap.end()){
        std::cerr << __FUNCTION__ << m_actContactState.data[i].link2 << " not found" << std::endl;
        continue;
      }
      this->contacts[numContact]->link2 = this->linkNameMap[std::string(m_actContactState.data[i].link2)];
      this->contacts[numContact]->freeX = m_actContactState.data[i].free_x;
      this->contacts[numContact]->freeY = m_actContactState.data[i].free_y;
      numContact++;
    }
    this->contacts.resize(numContact);
  }

};
