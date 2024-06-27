#ifndef ACTKINBALANCER_FOOTSTEPGENERATOR_H
#define ACTKINBALANCER_FOOTSTEPGENERATOR_H

#include <cnoid/Body>
#include "State.h"
#include "Goal.h"
#include "Output.h"

namespace actkin_balancer {

  class FootStepGenerator {
  public:
    void onStartBalancer(const State& state);
    void calcFootSteps(const State& state, const Goal& goal, const std::string& instance_name, double dt,
                       Output& output) const;

    int debugLevel = 3;
  protected:
    void calcRealStrideLimitationHull(const int& swingLeg, const double& theta, const State& state, const std::vector<Eigen::Vector2d>& strideLimitationHull,
                                      std::vector<Eigen::Vector2d>& realStrideLimitationHull) const;

    class FootStepCandidate {
    public:
      int supportLeg = NUM_LEGS; // RLEG or LLEG or NUM_LEGS(両足)
      int minDoubleTimeIdx = 0; // はじめdoubleTimeだけ両足支持で待つ. supportLeg==NUM_LEGSなら使用しない.
      int maxDoubleTimeIdx = 0; // はじめdoubleTimeだけ両足支持で待つ. supportLeg==NUM_LEGSなら使用しない.
      cnoid::Vector3 p = cnoid::Vector3::Zero(); // swingLegの着地位置. 支持脚相対. 要素数は1以上. supportLeg==NUM_LEGSなら使用しない. Zの値は変更される
      double theta = 0.0; // swingLegの着地角度. 支持脚相対. supportLeg==NUM_LEGSなら使用しない
      cnoid::Isometry3 pose = cnoid::Isometry3::Identity(); // 支持脚相対. pとthetaと重複. Zの値は変更される
      Eigen::Isometry2d pose2D = Eigen::Isometry2d::Identity(); // 支持脚相対. pとthetaと重複
      double minTime = 0.0;
      double maxTime = 0.0; // 要素数2. swingLegがdetachしてから着地するまでの時間の領域. supportLeg==NUM_LEGSなら使用しない. 変更される
      bool down = false;

      int xIdx = 0;
      int yIdx = 0;
      int thetaIdx = 0;
    };

    std::shared_ptr<FootStepCandidate> initialCandidateBoth; // supportLegがNUM_LEGS
    std::vector<std::vector<std::shared_ptr<FootStepCandidate> > > initialCandidates = std::vector<std::vector<std::shared_ptr<FootStepCandidate> > >(NUM_LEGS); // supportLegがrleg/lleg. keepDoubleはfalse

    std::vector<std::vector<double> > sampledX=std::vector<std::vector<double> >(NUM_LEGS); // swingLegがrleg/lleg. 支持脚相対
    std::vector<std::vector<double> > sampledY=std::vector<std::vector<double> >(NUM_LEGS); // swingLegがrleg/lleg. 支持脚相対
    mutable std::vector<std::vector<double> > sampledTheta=std::vector<std::vector<double> >(NUM_LEGS); // swingLegがrleg/lleg. 支持脚相対. 0番目がcurrent. 1番目がdefault
    std::vector<double> sampledTime;
    std::vector<double> sampledDoubleTime;

    mutable std::shared_ptr<FootStepCandidate> prevTarget = nullptr;

    inline void calcPath(const State& state, const std::shared_ptr<FootStepCandidate>& target, double refTime, const std::vector<cnoid::Isometry3>& legCoords/*world frame*/, bool timeOnly, bool forceDown, const std::vector<Eigen::Isometry2d>& legCoords2D, const std::vector<cnoid::Isometry3>& legCoordsHorizontal,
                         std::vector<cnoid::Isometry3>& path, std::vector<double>& time, bool& nearContact) const;
    inline bool onTarget(const State& state, const std::shared_ptr<FootStepCandidate>& target, const std::vector<cnoid::Isometry3>& legCoords/*world frame*/, const std::vector<Eigen::Isometry2d>& legCoords2D, const std::vector<cnoid::Isometry3>& legCoordsHorizontal) const;

    void print(const std::vector<std::shared_ptr<FootStepCandidate> >& candidates) const;
  };

};

#endif
