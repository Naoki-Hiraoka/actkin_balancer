#ifndef ACTKINBALANCER_GOAL_H
#define ACTKINBALANCER_GOAL_H

#include "State.h"
#include <cpp_filters/cpp_filters.h>
#include <actkin_balancer_msgs/idl/ActKinBalancer.hh>

namespace actkin_balancer{
  class RefRB {
  public:
    std::vector<cnoid::Isometry3> rb; // 必ずサイズは1以上. Z軸は鉛直

    double xyGoalTorelance = 0.1;
    double yawGoalTorelance = 0.1;
  public:
    static bool calcRBCoords(const State& state, cnoid::Isometry3& coords);
  };

  class Goal {
  public:
    // from port
    std::vector<std::shared_ptr<RefRB> > rbGoals;

  public:
    // RTC起動時に一回呼ばれる.
    void init(const State& state);

    // startStabilizer時に呼ばれる
    void onStartBalancer();

    // MODE_ST中のみ呼ばれる
    void updateFromIdl(const State& state, const actkin_balancer_msgs::RefStateIdl& m_refState);
    void updateFromIdl(const State& state, const actkin_balancer_msgs::RefRBIdl& m_refVRP);

    // MODE_ABC中のみ呼ばれる. 各goalをdtだけ補間する.
    void interpolate(const State& state, double dt);

  };

};

#endif
