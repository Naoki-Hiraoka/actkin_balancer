#ifndef ACTKINBALANCER_STATE_H
#define ACTKINBALANCER_STATE_H

#include <contact_state_msgs/idl/ContactState.hh>
#include <unordered_map>
#include <memory>
#include <cnoid/Body>

namespace actkin_balancer{
  class Contact {
  public:
    cnoid::LinkPtr link1;
    cnoid::Isometry3 localPose1 = cnoid::Isometry3::Identity();
    cnoid::LinkPtr link2;
    bool freeX = false;
    bool freeY = false;
    //std::shared_ptr<ik_constraint2::PositionConstraint> ikc;
  };

  // このクラスのメンバ変数は、全てfiniteである(nanやinfが無い)ことが仮定されている. 外部から値をセットするときには、finiteでない値を入れないようにすること
  class State {
  public:
    // from data port. 狭義のstate
    cnoid::BodyPtr robot; // actual.
    cnoid::Vector3 cogVel = cnoid::Vector3::Zero();
    std::vector<std::shared_ptr<Contact> > contacts; // actual

    // objects
  public:
    std::unordered_map<std::string, cnoid::LinkPtr> linkNameMap; // MODE_ABC中はconstant. URDFのLink名 -> linkPtr

    // RTC起動時に一回呼ばれる.
    void init(const cnoid::BodyPtr& robot_);

    // startBalancer時に呼ばれる
    void onStartBalancer();

    // MODE_ST中のみ呼ばれる
    void updateRobotFromIdl(const RTC::TimedDoubleSeq& m_qAct, const RTC::TimedDoubleSeq& m_dqAct, const RTC::TimedPose3D& m_actBasePose, const RTC::TimedVelocity3D& m_actBaseVel, double dt);
    void updateContactFromIdl(const contact_state_msgs::TimedContactSeq& m_actContactState);
  };

};

#endif
