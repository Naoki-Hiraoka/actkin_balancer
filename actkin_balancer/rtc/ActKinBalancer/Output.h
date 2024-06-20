#ifndef ACTKINBALANCER_OUTPUT_H
#define ACTKINBALANCER_OUTPUT_H

#include <actkin_stabilizer_msgs/idl/ActKinStabilizer.hh>
#include <cnoid/Body>
#include "State.h"

namespace actkin_balancer {
  class RefEEPoint {
  public:
    double time = 0.0;
    cnoid::Isometry3 pose = cnoid::Isometry3::Identity();
    cnoid::Vector6 velocity = cnoid::Vector6::Zero();
    cnoid::Vector6 wrench = cnoid::Vector6::Zero();
  };
  class RefEE {
  public:
    std::string name;
    cnoid::LinkPtr link;
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity();

    cnoid::LinkPtr frameLink;
    cnoid::Isometry3 framePose = cnoid::Isometry3::Identity();
    std::vector<RefEEPoint> trajectory; // 必ずサイズは1以上
    std::vector<bool> freeAxis = std::vector<bool>(6,true);
    int priority = 1; // 0 or 1. 0ならふつう. 1は重心と同じ
  };

  class RefVRPPoint{
  public:
    double time = 0.0;
    cnoid::Vector3 point = cnoid::Vector3::Zero();
  };
  class RefVRP {
  public:
    double omega = 1.0;
    std::vector<RefVRPPoint> trajectory; // 必ずサイズは1以上
  };

  class RefqPoint {
  public:
    double time;
    cnoid::VectorX q;
    cnoid::VectorX dq;
  };
  class Refq {
  public:
    std::vector<RefqPoint> trajectory; // 必ずサイズは1以上
  };

  class RefContact {
  public:
    std::string name;
    cnoid::LinkPtr link1;
    cnoid::Isometry3 localPose1 = cnoid::Isometry3::Identity();
    cnoid::LinkPtr link2;
    std::vector<bool> freeAxis = std::vector<bool>(6,false); // localPose1 local
    Region3D region; // localPose1 local

    Eigen::MatrixXd wrenchC; // localPose1 frame/origin. link1がlink2から受ける力に関する接触力制約.
    cnoid::VectorX wrenchld;
    cnoid::VectorX wrenchud;

    cnoid::Isometry3 localPose2 = cnoid::Isometry3::Identity();
  };

  enum class Feasibility {
    FEASIBLE,
    UNPREFERABLE,
    POSTPONED,
    INFEASIBLE
  };

  class Output {
  public:
    std::vector<RefEE> eeGoals;
    std::vector<RefVRP> vrpGoals;
    std::vector<Refq> qGoals;
    std::vector<RefContact> contactGoals;
    Feasibility feasibility = Feasibility::INFEASIBLE;

    void convertToIdl(const State& state, actkin_stabilizer_msgs::RefStateIdl& m_refState) const;
  };
};

#endif
