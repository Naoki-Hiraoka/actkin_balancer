#ifndef ACTKINBALANCER_FOOTSTEPGENERATOR_H
#define ACTKINBALANCER_FOOTSTEPGENERATOR_H

#include <cnoid/Body>
#include "State.h"
#include "Goal.h"
#include "Output.h"

namespace actkin_balancer {

  class FootStepGenerator {
  public:
    void calcFootSteps(const State& state, const Goal& goal, const std::string& instance_name, double dt,
                       Output& output) const;

  protected:
    void calcDefaultStep(int swingLeg, const State& state, const Goal& goal,
                         cnoid::Isometry3& landingCoords);
    void calcRealStrideLimitationHull(const int& swingLeg, const double& theta, const State& state, const std::vector<Eigen::Vector2d>& strideLimitationHull,
                                      std::vector<Eigen::Vector2d>& realStrideLimitationHull) const;
    void calcPath(int swingLeg, const State& state, const cnoid::Isometry3& target/*支持脚相対*/,
                  std::vector<cnoid::Vector3>& path, std::vector<double>& time) const;

    class FootStepCandidate {
    public:
      int supportLeg = NUM_LEGS; // RLEG or LLEG or NUM_LEGS(両足)
      bool keepDouble = true; // CPがsupportLegに乗るまで両足支持で待つ. supportLeg==NUM_LEGSなら使用しない
      cnoid::Vector3 p; // swingLegの着地位置. 支持脚相対. 要素数は1以上. supportLeg==NUM_LEGSなら使用しない
      double theta = 0.0; // swingLegの着地角度. 支持脚相対. supportLeg==NUM_LEGSなら使用しない
      double minTime = 0.0;
      double maxTime = 0.0; // 要素数2. swingLegがdetachしてから着地するまでの時間の領域. supportLeg==NUM_LEGSなら使用しない

    };
  };

};

#endif
