#include "FootStepGenerator.h"
#include "MathUtil.h"
#include <cnoid/EigenUtil>
#include <cnoid/TimeMeasure>
#include <iomanip>

namespace actkin_balancer{

  void FootStepGenerator::onStartBalancer(const State& state){
    this->prevTarget = nullptr;

    this->initialCandidateBoth = std::make_shared<FootStepCandidate>();
    this->initialCandidateBoth->supportLeg = NUM_LEGS;

    this->initialCandidates.clear();
    this->initialCandidates.resize(NUM_LEGS);


    {
      // timeをsample
      this->sampledTime.clear();
      for(double t = 0.0; t<=state.ee[RLEG].maxSwingTime;t+=state.ee[RLEG].resolutionTime){
        this->sampledTime.push_back(t);
      }
    }

    {
      // doubletimeをsample
      this->sampledDoubleTime.clear();
      for(double t = 0.0; t<=state.ee[RLEG].maxDoubleTime;t+=state.ee[RLEG].resolutionTime){
        this->sampledDoubleTime.push_back(t);
      }
    }

    for(int supportLeg=0;supportLeg<NUM_LEGS;supportLeg++){
      int swingLeg = (supportLeg == RLEG) ? LLEG : RLEG;

      {
        // thetaをsample
        this->sampledTheta[swingLeg].clear(); // 支持脚相対
        this->sampledTheta[swingLeg].push_back(0.0); // 0: current
        this->sampledTheta[swingLeg].push_back(0.0); // 1: default
      }

      {
        // Xをsample.
        this->sampledX[swingLeg].clear(); // 支持脚相対
        std::vector<Eigen::Vector2d> tmp;
        double minX = - mathutil::findExtremes(state.ee[swingLeg].strideLimitationHull, -Eigen::Vector2d::UnitX(), tmp);
        double maxX = mathutil::findExtremes(state.ee[swingLeg].strideLimitationHull, Eigen::Vector2d::UnitX(), tmp);
        cnoid::Vector3 origin = state.ee[swingLeg].defaultTranslatePos - state.ee[supportLeg].defaultTranslatePos;
        for(double x=origin[0];x<=maxX;x+=state.ee[swingLeg].resolutionXY){
          if(x < minX) continue;
          this->sampledX[swingLeg].push_back(x);
        }
        for(double x=origin[0]-state.ee[swingLeg].resolutionXY;x>=minX;x-=state.ee[swingLeg].resolutionXY){
          if(x > maxX) continue;
          this->sampledX[swingLeg].insert(this->sampledX[swingLeg].begin(),x);
        }
      }


      {
        // Yをsample.
        this->sampledY[swingLeg].clear(); // 支持脚相対
        std::vector<Eigen::Vector2d> tmp;
        double minY = - mathutil::findExtremes(state.ee[swingLeg].strideLimitationHull, -Eigen::Vector2d::UnitY(), tmp);
        double maxY = mathutil::findExtremes(state.ee[swingLeg].strideLimitationHull, Eigen::Vector2d::UnitY(), tmp);
        cnoid::Vector3 origin = state.ee[swingLeg].defaultTranslatePos - state.ee[supportLeg].defaultTranslatePos;
        for(double y=origin[1];y<=maxY;y+=state.ee[swingLeg].resolutionXY){
          if(y < minY) continue;
          this->sampledY[swingLeg].push_back(y);
        }
        for(double y=origin[1]-state.ee[swingLeg].resolutionXY;y>=minY;y-=state.ee[swingLeg].resolutionXY){
          if(y > maxY) continue;
          this->sampledY[swingLeg].insert(this->sampledY[swingLeg].begin(),y);
        }
      }

      // stride limitationとZ高さのみ考慮してcandidateを作る
      for(int i=0;i<this->sampledTheta[swingLeg].size();i++){
        double theta = this->sampledTheta[swingLeg][i];
        for(int x=0;x<this->sampledX[swingLeg].size();x++){
          for(int y=0;y<this->sampledY[swingLeg].size();y++){
            Eigen::Vector2d p(this->sampledX[swingLeg][x],this->sampledY[swingLeg][y]);
            std::shared_ptr<FootStepCandidate> candidate = std::make_shared<FootStepCandidate>();
            candidate->supportLeg = supportLeg;
            candidate->minDoubleTimeIdx = 0;
            candidate->maxDoubleTimeIdx = int(this->sampledDoubleTime.size()) - 1;
            candidate->p.head<2>() = p;
            candidate->xIdx = x;
            candidate->yIdx = y;
            candidate->theta = theta;
            candidate->thetaIdx = i;
            candidate->pose.translation() = candidate->p;
            candidate->pose.linear() = cnoid::AngleAxisd(candidate->theta, cnoid::Vector3::UnitZ()).toRotationMatrix();
            candidate->pose2D.translation() = candidate->p.head<2>();
            candidate->pose2D.linear() = Eigen::Rotation2Dd(candidate->theta).toRotationMatrix();
            candidate->minTime = 0.0;
            candidate->maxTime = state.ee[swingLeg].maxSwingTime;
            candidate->down = false;
            this->initialCandidates[supportLeg].push_back(candidate);
          }
        }
      }
    }

  }

  void FootStepGenerator::calcFootSteps(const State& state, const Goal& goal, const std::string& instance_name, double dt,
                                        Output& output) const{
    cnoid::TimeMeasure timer;
    if(this->debugLevel>=1) timer.begin();

    // 両足とも接地していなければinfeasible
    if(!state.actContact[RLEG] && !state.actContact[LLEG]) {
      output.feasibility = Feasibility::INFEASIBLE;
      output.eeGoals.clear();
      output.vrpGoals.clear();
      output.qGoals.clear();
      output.contactGoals.clear();
      return;
    }

    // 歩行可能姿勢のチェック -> UNPREFERABLE
    bool preferable = true;
    // TODO

    std::vector<cnoid::Isometry3> legCoords{state.ee[RLEG].parentLink->T() * state.ee[RLEG].localPose, state.ee[LLEG].parentLink->T() * state.ee[LLEG].localPose}; // world frame
    std::vector<cnoid::Isometry3> legCoordsHorizontal{
      mathutil::orientCoordToAxis(legCoords[RLEG], cnoid::Vector3::UnitZ()),
      mathutil::orientCoordToAxis(legCoords[LLEG], cnoid::Vector3::UnitZ())};  // world frame
    std::vector<Eigen::Isometry2d> legCoords2D(NUM_LEGS); // world frame
    for(int leg=0;leg<NUM_LEGS;leg++){
      legCoords2D[leg].translation() = legCoordsHorizontal[leg].translation().head<2>();
      legCoords2D[leg].linear() = legCoordsHorizontal[leg].linear().topLeftCorner<2,2>();
    }

    // 現在のnominalを求める
    const NominalInfo& nominal = state.nominal;

    // defaultを求める
    std::vector<Eigen::Vector2d> defaultp(NUM_LEGS); // swingLegがRLEG / LLEG. 支持脚相対
    std::vector<double> defaultTheta(NUM_LEGS); // swingLegがRLEG / LLEG. 支持脚相対
    for(int swingLeg = 0; swingLeg < NUM_LEGS; swingLeg++){
      int supportLeg = swingLeg==RLEG ? LLEG : RLEG;

      if(goal.rbGoals.size() > 0){
        cnoid::Isometry3 target = goal.rbGoals[0]->rb[0]; // world frame
        target.translation() = target * state.ee[swingLeg].defaultTranslatePos;
        cnoid::Isometry3 trans = legCoordsHorizontal[supportLeg].inverse() * target; // 支持脚相対

        double theta = cnoid::rpyFromRot(trans.linear())[2];
        theta = std::min(state.ee[swingLeg].strideLimitationMaxTheta,
                         std::max(state.ee[swingLeg].strideLimitationMinTheta,
                                  theta));
        std::vector<Eigen::Vector2d> realDefaultStrideLimitationHull; // 支持脚相対
        this->calcRealStrideLimitationHull(swingLeg, theta, state, state.ee[swingLeg].defaultStrideLimitationHull,
                                           realDefaultStrideLimitationHull);
        Eigen::Vector2d p = mathutil::calcNearestPointOfHull(trans.translation().head<2>(), realDefaultStrideLimitationHull);
        defaultp[swingLeg] = p;
        defaultTheta[swingLeg] = theta;
      }else{
        defaultp[swingLeg] = (- state.ee[supportLeg].defaultTranslatePos + state.ee[swingLeg].defaultTranslatePos).head<2>();
        defaultTheta[swingLeg] = 0.0;
      }
    }

    if(this->debugLevel >= 1) std::cerr << "start: " << timer.measure() << "[s]." << std::endl;

    // 候補
    std::vector<std::shared_ptr<FootStepCandidate>> candidates;
    std::vector<std::shared_ptr<FootStepCandidate>> nextCandidates;


    if(this->prevTarget &&
       this->prevTarget->supportLeg != NUM_LEGS &&
       this->prevTarget->down // 結局blindに接触するのが一番安定する.
       ) {
      int supportLeg = this->prevTarget->supportLeg;
      int swingLeg = (supportLeg == RLEG) ? LLEG : RLEG;
      if(state.actContact[supportLeg] && !state.actContact[swingLeg]) {
        std::vector<cnoid::Isometry3> path;
        std::vector<double> time;
        bool nearContact;
        this->calcPath(state, this->prevTarget, std::max(0.0,this->prevTarget->minTime-dt), legCoords, true, this->prevTarget->down, legCoords2D, legCoordsHorizontal,
                       path, time, nearContact);
        this->prevTarget->minDoubleTimeIdx = 0;
        this->prevTarget->maxDoubleTimeIdx = 0;
        this->prevTarget->maxTime = time.back();
        this->prevTarget->minTime = time.back();
        candidates.push_back(this->prevTarget);
      }
    }

    if(candidates.size() == 0){

      // stride limitationで初期化. minTimeとmaxTimeを初期化
      {
        candidates.reserve(this->initialCandidates[RLEG].size() + this->initialCandidates[LLEG].size() + 1);

        // thetaをリサンプリング
        for(int swingLeg=0;swingLeg<NUM_LEGS;swingLeg++){
          int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
          double currentTheta = Eigen::Rotation2Dd(legCoords2D[supportLeg].linear().inverse() * legCoords2D[swingLeg].linear()).smallestAngle(); // 支持脚相対
          this->sampledTheta[swingLeg][0] = currentTheta;
          this->sampledTheta[swingLeg][1] = defaultTheta[swingLeg];
        }

        if(state.actContact[RLEG] && state.actContact[LLEG]) {
          candidates.push_back(this->initialCandidateBoth);
        }

        for(int supportLeg=0;supportLeg<NUM_LEGS;supportLeg++){
          if(!state.actContact[supportLeg]) continue;
          int swingLeg = (supportLeg == RLEG) ? LLEG : RLEG;

          for(int i=0;i<this->initialCandidates[supportLeg].size();i++){
            std::shared_ptr<FootStepCandidate> candidate = this->initialCandidates[supportLeg][i];
            double theta = this->sampledTheta[swingLeg][candidate->thetaIdx];
            std::vector<Eigen::Vector2d> realStrideLimitationHull; // 支持脚相対
            this->calcRealStrideLimitationHull(swingLeg, theta, state, state.ee[swingLeg].strideLimitationHull,
                                               realStrideLimitationHull);
            if(!mathutil::isInsideHull(candidate->p.head<2>(),realStrideLimitationHull)) continue;

            candidate->theta = theta;
            candidate->pose.linear() = cnoid::AngleAxisd(candidate->theta, cnoid::Vector3::UnitZ()).toRotationMatrix();
            candidate->pose2D.linear() = Eigen::Rotation2Dd(candidate->theta).toRotationMatrix();

            candidate->minTime = 0.0;
            candidate->maxTime = state.ee[swingLeg].maxSwingTime;
            candidate->minDoubleTimeIdx = 0;
            if(state.actContact[swingLeg]) candidate->maxDoubleTimeIdx = int(this->sampledDoubleTime.size()) - 1;
            else candidate->maxDoubleTimeIdx = 0;
            candidate->down = false;
            candidates.push_back(candidate);
          }
        }

        nextCandidates.reserve(candidates.size());
      }

      if(debugLevel >= 2) {
        std::vector<double> num(3,0);
        for(int i=0;i<candidates.size();i++){
          num[candidates[i]->supportLeg]++;
        }
        std::cerr << "stride limitation " << candidates.size() << " ";
        for(int i=0;i<num.size();i++) std::cerr << num[i] << " ";
        std::cerr << std::endl;
      }
      if(debugLevel >= 3){
        this->print(candidates);
      }
      if(this->debugLevel >= 1) std::cerr << "stride limitation: " << timer.measure() << "[s]." << std::endl;


      // Z情報を初期化・付与し、Z高さで絞り込み
      {
        nextCandidates.clear();

        for(int i=0;i<candidates.size();i++){
          if(candidates[i]->supportLeg == NUM_LEGS) {
            nextCandidates.push_back(candidates[i]);
            continue;
          }

          int swingLeg = (candidates[i]->supportLeg == RLEG) ? LLEG : RLEG;

          double z = 0.0;
          // TODO. heightmapからzを計算.

          if(z <= state.ee[swingLeg].maxLandingHeight &&
             z >= state.ee[swingLeg].minLandingHeight){
            std::shared_ptr<FootStepCandidate> nextCandidate = candidates[i];
            nextCandidate->p[2] = z;
            nextCandidate->pose.translation()[2] = z;
            nextCandidates.push_back(nextCandidate);
          }
        }

        if(nextCandidates.size() > 0.0) {
          candidates = nextCandidates;
        }else{
          std::cerr << "[" << instance_name << "] height not found" << std::endl;
        }
      }

      if(debugLevel >= 2) {
        std::vector<double> num(3,0);
        for(int i=0;i<candidates.size();i++){
          num[candidates[i]->supportLeg]++;
        }
        std::cerr << "Z " << candidates.size() << " ";
        for(int i=0;i<num.size();i++) std::cerr << num[i] << " ";
        std::cerr << std::endl;
      }
      if(this->debugLevel >= 1) std::cerr << "Z: " << timer.measure() << "[s]." << std::endl;

      // reachable時間で絞り込み
      {
        nextCandidates.clear();

        for(int i=0;i<candidates.size();i++){
          if(candidates[i]->supportLeg == NUM_LEGS) {
            nextCandidates.push_back(candidates[i]);
            continue;
          }

          int supportLeg = candidates[i]->supportLeg;
          int swingLeg = (candidates[i]->supportLeg == RLEG) ? LLEG : RLEG;

          std::vector<cnoid::Isometry3> path;
          std::vector<double> time;
          bool nearContact;
          this->calcPath(state, candidates[i], 0.0, legCoords, true, false, legCoords2D, legCoordsHorizontal,
                         path, time, nearContact);
          double minTime = time.back();

          double nextMinTime = candidates[i]->minTime;
          double nextMaxTime = candidates[i]->maxTime;

          if(minTime <= nextMinTime){
            // OK
          }else if(minTime <= nextMaxTime){
            nextMinTime = minTime;
          }else{
            continue;
          }

          std::shared_ptr<FootStepCandidate> nextCandidate = candidates[i];
          nextCandidate->minTime = nextMinTime;
          nextCandidate->maxTime = nextMaxTime;
          nextCandidate->down = nearContact;
          nextCandidates.push_back(nextCandidate);
        }

        if(nextCandidates.size() > 0.0) {
          candidates = nextCandidates;
        }else{
          std::cerr << "[" << instance_name << "] Reachiable not found" << std::endl;
        }
      }

      if(debugLevel >= 2) {
        std::vector<double> num(3,0);
        for(int i=0;i<candidates.size();i++){
          num[candidates[i]->supportLeg]++;
        }
        std::cerr << "reachiable " << candidates.size() << " ";
        for(int i=0;i<num.size();i++) std::cerr << num[i] << " ";
        std::cerr << std::endl;
      }
      if(debugLevel >= 3){
        this->print(candidates);
      }
      if(this->debugLevel >= 1) std::cerr << "reachiable: " << timer.measure() << "[s]." << std::endl;

      // steppableで絞り込み
      {
        nextCandidates.clear();
        for(int i=0;i<candidates.size();i++){
          if(candidates[i]->supportLeg == NUM_LEGS) {
            nextCandidates.push_back(candidates[i]);
            continue;
          }

          // steppableかどうかを判定する TODO
          std::shared_ptr<FootStepCandidate> nextCandidate = candidates[i];
          nextCandidates.push_back(nextCandidate);

        }

        if(nextCandidates.size() > 0.0) {
          candidates = nextCandidates;
        }else{
          std::cerr << "[" << instance_name << "] steppable not found" << std::endl;
        }
      }

      if(debugLevel >= 2) {
        std::vector<double> num(3,0);
        for(int i=0;i<candidates.size();i++){
          num[candidates[i]->supportLeg]++;
        }
        std::cerr << "steppable " << candidates.size() << " ";
        for(int i=0;i<num.size();i++) std::cerr << num[i] << " ";
        std::cerr << std::endl;
      }
      if(this->debugLevel >= 1) std::cerr << "steppable: " << timer.measure() << "[s]." << std::endl;

      // capturableで絞り込み
      std::vector<std::vector<Eigen::Vector2d> > supportHullBoth(NUM_LEGS); // RLEG相対/LLEG相対
      for(int leg=0;leg<NUM_LEGS;leg++){
        int leg2 = (leg == RLEG) ? LLEG : RLEG;
        for(int v=0;v<state.surface[leg].size();v++){
          supportHullBoth[leg].push_back(state.surface[leg][v]);
        }
        for(int v=0;v<state.surface[leg2].size();v++){
          supportHullBoth[leg].push_back(legCoords2D[leg].inverse() * legCoords2D[leg2] * state.surface[leg2][v]);
        }
        mathutil::calcConvexHull(supportHullBoth[leg],supportHullBoth[leg]);
      }

      std::vector<std::vector<std::vector<Eigen::Vector2d> > > startdcms(NUM_LEGS); // supportLegがrleg/lleg. doubleTime. 支持脚相対
      {
        for(int supportLeg=0;supportLeg<NUM_LEGS;supportLeg++){
          startdcms[supportLeg].resize(this->sampledDoubleTime.size());
          Eigen::Vector2d dcm = legCoords2D[supportLeg].inverse() * (state.robot->centerOfMass() + state.cogVel / std::sqrt(state.g / nominal.nominalZ)).head<2>(); // 支持脚相対
          for(int j=0;j<this->sampledDoubleTime.size();j++){
            std::vector<Eigen::Vector2d> startdcm; // 支持脚相対. 離地時
            if(this->sampledDoubleTime[j]==0){
              startdcm.push_back(dcm);
            }else{
              for(int v=0;v<supportHullBoth[supportLeg].size();v++){
                Eigen::Vector2d vrp = supportHullBoth[supportLeg][v];
                startdcm.push_back((dcm - vrp) * std::exp(std::sqrt(state.g / nominal.nominalZ) * this->sampledDoubleTime[j]) + vrp);
              }
            }
            mathutil::calcConvexHull(startdcm,startdcm);
            startdcms[supportLeg][j] = startdcm;
          }
        }
      }
      {
        nextCandidates.clear();

        for(int i=0;i<candidates.size();i++){
          if(candidates[i]->supportLeg == NUM_LEGS) {
            // 現在の両足支持姿勢が0-step capturableか
            Eigen::Vector2d dcm = legCoords2D[RLEG].inverse() * (state.robot->centerOfMass() + state.cogVel / std::sqrt(state.g / nominal.nominalZ)).head<2>(); // RLEG相対
            if(!mathutil::isInsideHull(dcm, supportHullBoth[RLEG])) continue;

            nextCandidates.push_back(candidates[i]);
            continue;
          }

          // 着地時にcapturableかどうかを判定する. 遊脚は着地位置のみ
          int supportLeg = candidates[i]->supportLeg;
          int swingLeg = (candidates[i]->supportLeg == RLEG) ? LLEG : RLEG;

          std::vector<Eigen::Vector2d> endSupportHull; // 支持脚相対. 着時時
          {
            for(int v=0;v<state.surface[supportLeg].size();v++){
              endSupportHull.push_back(state.surface[supportLeg][v]);
            }
            Eigen::Isometry2d pose = candidates[i]->pose2D;
            endSupportHull.push_back(pose * state.ee[swingLeg].copOffset + Eigen::Vector2d(state.ee[swingLeg].resolutionXY/2,state.ee[swingLeg].resolutionXY/2));
            endSupportHull.push_back(pose * state.ee[swingLeg].copOffset + Eigen::Vector2d(-state.ee[swingLeg].resolutionXY/2,state.ee[swingLeg].resolutionXY/2));
            endSupportHull.push_back(pose * state.ee[swingLeg].copOffset + Eigen::Vector2d(-state.ee[swingLeg].resolutionXY/2,-state.ee[swingLeg].resolutionXY/2));
            endSupportHull.push_back(pose * state.ee[swingLeg].copOffset + Eigen::Vector2d(state.ee[swingLeg].resolutionXY/2,-state.ee[swingLeg].resolutionXY/2));
            mathutil::calcConvexHull(endSupportHull,endSupportHull);
          }

          double minDistance = std::numeric_limits<double>::max();
          for(int s=candidates[i]->minDoubleTimeIdx;s<=candidates[i]->maxDoubleTimeIdx;s++){

            std::vector<Eigen::Vector2d> enddcm; // 支持脚相対. 着時時
            for(int v=0;v<state.surface[supportLeg].size();v++){
              Eigen::Vector2d vrp = state.surface[supportLeg][v];
              for(int d=0;d<startdcms[supportLeg][s].size();d++){
                enddcm.push_back((startdcms[supportLeg][s][d] - vrp) * std::exp(std::sqrt(state.g / nominal.nominalZ) * candidates[i]->minTime) + vrp);
              }
            }
            mathutil::calcConvexHull(enddcm,enddcm);

            double distance;
            if(mathutil::isIntersectConvexHull(enddcm, endSupportHull)) distance = 0.0;
            else distance = mathutil::calcDistanceOfTwoHull(enddcm, endSupportHull);

            if(distance > minDistance) break;
            minDistance = distance;
            candidates[i]->minDoubleTimeIdx = s;
            if(minDistance == 0) break;
          }

          if(minDistance == 0) nextCandidates.push_back(candidates[i]);

          // std::vector<double> capturableTime;
          // // minTimeの場合
          // int minTimeIdx = std::min(int(this->sampledTime.size())-1, std::max(0,int(candidates[i]->minTime / state.ee[swingLeg].resolutionTime))); // minTimeより小さい最大のtime
          // bool intersect1 = mathutil::isIntersectConvexHull(enddcms[supportLeg][candidates[i]->doubleTimeIdx][minTimeIdx], endSupportHulls[supportLeg][candidates[i]->xIdx][candidates[i]->yIdx][candidates[i]->thetaIdx]);
          // int minTimeIdx2 = std::min(minTimeIdx+1, int(this->sampledTime.size())-1); // minTimeより大きい最小のtime
          // if(intersect1){
          //   bool intersect2 = mathutil::isIntersectConvexHull(enddcms[supportLeg][candidates[i]->doubleTimeIdx][minTimeIdx2], endSupportHulls[supportLeg][candidates[i]->xIdx][candidates[i]->yIdx][candidates[i]->thetaIdx]);
          //   if(intersect2) {
          //     capturableTime.push_back(candidates[i]->minTime);
          //     capturableTime.push_back(this->sampledTime[minTimeIdx2]);
          //   }
          // }

          // if(capturableTime.size() != 0){ // minTimeがcapturableでないなら以後capturableになることはない
          //   // maxTimeの場合
          //   int maxTimeIdx = std::min(int(this->sampledTime.size())-1, int(std::max(0.0,candidates[i]->maxTime / state.ee[swingLeg].resolutionTime)));
          //   bool intersect3 = mathutil::isIntersectConvexHull(enddcms[supportLeg][candidates[i]->doubleTimeIdx][maxTimeIdx], endSupportHulls[supportLeg][candidates[i]->xIdx][candidates[i]->yIdx][candidates[i]->thetaIdx]);
          //   if(intersect3) capturableTime.push_back(candidates[i]->maxTime);

          //   if(capturableTime.size() == 2){ // minTimeはOKだがmaxTimeはだめ
          //     for(int t=minTimeIdx2+1;t<maxTimeIdx;t++){
          //       bool intersect4 = mathutil::isIntersectConvexHull(enddcms[supportLeg][candidates[i]->doubleTimeIdx][t], endSupportHulls[supportLeg][candidates[i]->xIdx][candidates[i]->yIdx][candidates[i]->thetaIdx]);
          //       if(intersect4) capturableTime.push_back(this->sampledTime[t]);
          //       else break;
          //     }
          //   }

          //   candidates[i]->maxTime = capturableTime.back();
          //   nextCandidates.push_back(candidates[i]);
        }

        if(nextCandidates.size() > 0.0) {
          candidates = nextCandidates;
        }else{
          // 前回の解に最も近い位置に最速でおろす
          if(this->prevTarget &&
             this->prevTarget->supportLeg != NUM_LEGS){
            double minDistance = std::numeric_limits<double>::max();
            for(int i=0;i<candidates.size();i++){
              if(candidates[i]->supportLeg == this->prevTarget->supportLeg){
                double distance = (candidates[i]->p.head<2>() - this->prevTarget->p.head<2>()).norm();
                if(distance < minDistance ||
                   distance == minDistance && candidates[i]->thetaIdx == this->prevTarget->thetaIdx){
                  minDistance = distance;
                  nextCandidates.clear();
                  candidates[i]->maxDoubleTimeIdx = candidates[i]->minDoubleTimeIdx;
                  candidates[i]->maxTime = candidates[i]->minTime;
                  nextCandidates.push_back(candidates[i]);
                }
              }
            }
          }

          if(nextCandidates.size() > 0.0) {
            candidates = nextCandidates;
          }else{
            std::cerr << "[" << instance_name << "] capturable not found" << std::endl;
          }
        }
      }

      if(debugLevel >= 2) {
        std::vector<double> num(3,0);
        for(int i=0;i<candidates.size();i++){
          num[candidates[i]->supportLeg]++;
        }
        std::cerr << "capturable " << candidates.size() << " ";
        for(int i=0;i<num.size();i++) std::cerr << num[i] << " ";
        std::cerr << std::endl;
      }
      if(debugLevel >= 3){
        this->print(candidates);
      }
      if(this->debugLevel >= 1) std::cerr << "capturable: " << timer.measure() << "[s]." << std::endl;

      // default stepで絞り込み
      // stop
      if(candidates.size() > 1){
        for(int i=0;i<candidates.size();i++){
          if(candidates[i]->supportLeg == NUM_LEGS) {
            if(goal.rbGoals.size() == 0 ||
               (goal.rbGoals.size() == 1 && goal.rbGoals[0]->isSatisfied(state))){
              nextCandidates.clear();
              nextCandidates.push_back(candidates[i]);
            }
            break;
          }
        }
        if(nextCandidates.size() > 0.0) {
          candidates = nextCandidates;
        }
      }
      if(debugLevel >= 2) {
        std::vector<double> num(3,0);
        for(int i=0;i<candidates.size();i++){
          num[candidates[i]->supportLeg]++;
        }
        std::cerr << "stop " << candidates.size() << " ";
        for(int i=0;i<num.size();i++) std::cerr << num[i] << " ";
        std::cerr << std::endl;
      }
      if(this->debugLevel >= 1) std::cerr << "stop: " << timer.measure() << "[s]." << std::endl;

      // default stepで絞り込み.
      // thetaがcurerntよりもどれだけ近づくか
      if(candidates.size() > 1){
        double maxDistance = - std::numeric_limits<double>::max(); // 近づいた距離
        nextCandidates.clear();

        // thetaIdx: 0より1を好む.
        std::shared_ptr<FootStepCandidate> doubleSupportCandidate = nullptr;
        for(int i=0;i<candidates.size();i++){
          if(candidates[i]->supportLeg != NUM_LEGS) {
            if(candidates[i]->thetaIdx == 1) nextCandidates.push_back(candidates[i]);
          }else{
            doubleSupportCandidate = candidates[i]; //保留
          }
        }

        if(nextCandidates.size() > 0) {
          if( (std::abs(this->sampledTheta[RLEG][0] - this->sampledTheta[RLEG][1]) <= state.ee[RLEG].resolutionTheta) &&
              (std::abs(this->sampledTheta[LLEG][0] - this->sampledTheta[LLEG][1]) <= state.ee[LLEG].resolutionTheta) ) {
            if(doubleSupportCandidate)  nextCandidates.push_back(doubleSupportCandidate);
          }
          candidates = nextCandidates;

          // 1同士なら、resolutionTheta以上の差があるならR/Lの一方のみを好む.
          nextCandidates.clear();
          if( std::abs(std::abs(this->sampledTheta[RLEG][0] - this->sampledTheta[RLEG][1]) - std::abs(this->sampledTheta[LLEG][0] - this->sampledTheta[LLEG][1])) > state.ee[RLEG].resolutionTheta ) {
            int preferedLeg = (std::abs(this->sampledTheta[RLEG][0] - this->sampledTheta[RLEG][1]) > std::abs(this->sampledTheta[LLEG][0] - this->sampledTheta[LLEG][1])) ? LLEG : RLEG; // supportleg
            for(int i=0;i<candidates.size();i++){
              if(candidates[i]->supportLeg == preferedLeg) nextCandidates.push_back(candidates[i]);
            }
          }

          if(nextCandidates.size() > 0) {
            candidates = nextCandidates;
          }
        }
      }

      if(debugLevel >= 2) {
        std::vector<double> num(3,0);
        for(int i=0;i<candidates.size();i++){
          num[candidates[i]->supportLeg]++;
        }
        std::cerr << "default theta " << candidates.size() << " ";
        for(int i=0;i<num.size();i++) std::cerr << num[i] << " ";
        std::cerr << std::endl;
      }
      if(debugLevel >= 3){
        this->print(candidates);
      }
      if(this->debugLevel >= 1) std::cerr << "default theta: " << timer.measure() << "[s]." << std::endl;

      // default stepで絞り込み.
      // posがcurerntよりもどれだけ近づくか
      if(candidates.size() > 1){
        std::shared_ptr<FootStepCandidate> doubleSupportCandidate = nullptr;
        double maxDistance = - std::numeric_limits<double>::max(); // 近づいた距離
        nextCandidates.clear();
        for(int i=0;i<candidates.size();i++){
          if(candidates[i]->supportLeg == NUM_LEGS) {
            doubleSupportCandidate = candidates[i]; //保留
            continue;
          }

          int supportLeg = candidates[i]->supportLeg;
          int swingLeg = (candidates[i]->supportLeg == RLEG) ? LLEG : RLEG;
          Eigen::Vector2d currentp = legCoords2D[supportLeg].inverse() * legCoords2D[swingLeg].translation(); // 支持脚相対
          double currentError = (currentp - defaultp[swingLeg]).norm();
          Eigen::Vector2d endp = candidates[i]->p.head<2>(); // 支持脚相対
          double endError = (endp - defaultp[swingLeg]).norm();
          double distance = currentError - endError;

          if(distance > maxDistance + state.ee[swingLeg].resolutionXY/*epsilon*/){
            maxDistance = distance;
            nextCandidates.clear();
            nextCandidates.push_back(candidates[i]);
          }else if(distance > maxDistance - state.ee[swingLeg].resolutionXY/*epsilon*/){
            nextCandidates.push_back(candidates[i]);
          }
        }

        if(maxDistance < state.ee[RLEG].resolutionXY) {
          if(doubleSupportCandidate) nextCandidates.push_back(doubleSupportCandidate);
        }

        if(nextCandidates.size() > 0.0) {
          candidates = nextCandidates;
        }else{
          std::cerr << "[" << instance_name << "] default pos not found" << std::endl;
        }
      }

      if(debugLevel >= 2) {
        std::cerr << "default pos " << candidates.size() << std::endl;
      }
      if(debugLevel >= 3){
        this->print(candidates);
      }
      if(debugLevel >= 3) {
        for(int i=0;i<candidates.size();i++){
          std::cerr << candidates[i]->supportLeg << " " << candidates[i]->minTime << " " << candidates[i]->p.transpose() << " " << candidates[i]->theta << " " << candidates[i]->minDoubleTimeIdx <<  std::endl;
        }
      }
      if(this->debugLevel >= 1) std::cerr << "default pos: " << timer.measure() << "[s]." << std::endl;

      // prevTargetで絞り込む
      if(candidates.size() > 1){
        nextCandidates.clear();

        int preferedLeg; // supportleg
        if(this->prevTarget) {
          preferedLeg = this->prevTarget->supportLeg;
        }else{
          preferedLeg = NUM_LEGS;
        }
        double minDistance = std::numeric_limits<double>::max();
        for(int i=0;i<candidates.size();i++){
          if(candidates[i]->supportLeg == preferedLeg){
            double distance = (candidates[i]->p.head<2>() - this->prevTarget->p.head<2>()).norm();
            if(distance < minDistance){
              minDistance = distance;
              nextCandidates.clear();
              nextCandidates.push_back(candidates[i]);
            }
          }
        }

        if(nextCandidates.size() > 0.0) {
          candidates = nextCandidates;
        }
      }
      if(debugLevel >= 2) {
        std::cerr << "prev " << candidates.size() << std::endl;
      }
      if(debugLevel >= 3){
        this->print(candidates);
      }
      if(debugLevel >= 3) {
        for(int i=0;i<candidates.size();i++){
          std::cerr << candidates[i]->supportLeg << " " << candidates[i]->minTime << " " << candidates[i]->p.transpose() << " " << candidates[i]->theta << " " << candidates[i]->minDoubleTimeIdx <<  std::endl;
        }
      }
      if(this->debugLevel >= 1) std::cerr << "prev: " << timer.measure() << "[s]." << std::endl;

      // default stepで絞り込み.
      // default velocityの時間との誤差
      {
        double minError = std::numeric_limits<double>::max(); // defaultに要する時間で割った比
        nextCandidates.clear();
        for(int i=0;i<candidates.size();i++){
          if(candidates[i]->supportLeg == NUM_LEGS) {
            nextCandidates.clear();
            nextCandidates.push_back(candidates[i]);
            break;
          }

          int supportLeg = candidates[i]->supportLeg;
          int swingLeg = (candidates[i]->supportLeg == RLEG) ? LLEG : RLEG;
          cnoid::Isometry3 targetPose = legCoordsHorizontal[supportLeg] * candidates[i]->pose; // world frame
          double defaultTime = candidates[i]->minTime / state.ee[swingLeg].defaultSwingVelocityRatio;

          std::vector<Eigen::Vector2d> endSupportHull; // 支持脚相対. 着時時
          {
            for(int v=0;v<state.surface[supportLeg].size();v++){
              endSupportHull.push_back(state.surface[supportLeg][v]);
            }
            Eigen::Isometry2d pose = candidates[i]->pose2D;
            endSupportHull.push_back(pose * state.ee[swingLeg].copOffset + Eigen::Vector2d(state.ee[swingLeg].resolutionXY/2,state.ee[swingLeg].resolutionXY/2));
            endSupportHull.push_back(pose * state.ee[swingLeg].copOffset + Eigen::Vector2d(-state.ee[swingLeg].resolutionXY/2,state.ee[swingLeg].resolutionXY/2));
            endSupportHull.push_back(pose * state.ee[swingLeg].copOffset + Eigen::Vector2d(-state.ee[swingLeg].resolutionXY/2,-state.ee[swingLeg].resolutionXY/2));
            endSupportHull.push_back(pose * state.ee[swingLeg].copOffset + Eigen::Vector2d(state.ee[swingLeg].resolutionXY/2,-state.ee[swingLeg].resolutionXY/2));
            mathutil::calcConvexHull(endSupportHull,endSupportHull);
          }

          double maxDistance = - std::numeric_limits<double>::max();
          int maxs = candidates[i]->minDoubleTimeIdx;
          double maxMaxTime = candidates[i]->maxTime;
          double maxMinTime = candidates[i]->minTime;
          for(int s=candidates[i]->minDoubleTimeIdx;s<=candidates[i]->maxDoubleTimeIdx;s++){
            double minTime = candidates[i]->minTime;
            double maxTime = candidates[i]->minTime;
            for(double t=candidates[i]->minTime; t<=candidates[i]->maxTime; t+= state.ee[swingLeg].resolutionTime) {
              std::vector<Eigen::Vector2d> enddcm; // 支持脚相対. 着時時
              for(int v=0;v<state.surface[supportLeg].size();v++){
                Eigen::Vector2d vrp = state.surface[supportLeg][v];
                for(int d=0;d<startdcms[supportLeg][s].size();d++){
                  enddcm.push_back((startdcms[supportLeg][s][d] - vrp) * std::exp(std::sqrt(state.g / nominal.nominalZ) * t) + vrp);
                }
              }
              mathutil::calcConvexHull(enddcm,enddcm);

              if(!mathutil::isIntersectConvexHull(enddcm, endSupportHull)) break;
              maxTime = t;
            }

            if(maxTime - minTime > maxDistance){ // doubleTimeが小さい方を好む
              maxDistance = maxTime - minTime;
              maxs = s;
              maxMaxTime = maxTime;
              maxMinTime = minTime;
            }
          }

          candidates[i]->maxTime = maxMaxTime;
          candidates[i]->minTime = maxMinTime;
          candidates[i]->maxDoubleTimeIdx = maxs;
          candidates[i]->minDoubleTimeIdx = maxs;

          double error;
          if(defaultTime < candidates[i]->minTime) {
            error = candidates[i]->minTime - defaultTime;
            candidates[i]->maxTime = candidates[i]->minTime;
          }else if(defaultTime > candidates[i]->maxTime) {
            error = defaultTime - candidates[i]->maxTime;
            candidates[i]->minTime = candidates[i]->maxTime;
          }else{
            error = 0.0;
            candidates[i]->minTime = candidates[i]->maxTime = defaultTime;
          }

          if(error < minError - 0.001/*epsilon*/) {
            minError = error;
            nextCandidates.clear();
            nextCandidates.push_back(candidates[i]);
          }else if(error < minError + 0.001/*epsilon*/) {
            nextCandidates.push_back(candidates[i]);
          }
        }

        if(nextCandidates.size() > 0.0) {
          candidates = nextCandidates;
        }else{
          std::cerr << "[" << instance_name << "] default time not found" << std::endl;
        }
      }

      if(debugLevel >= 2) {
        std::cerr << "default time " << candidates.size() << std::endl;
      }
      if(debugLevel >= 3) {
        for(int i=0;i<candidates.size();i++){
          std::cerr << candidates[i]->supportLeg << " " << candidates[i]->minTime << " " << candidates[i]->p.transpose() << " " << candidates[i]->theta << " " << candidates[i]->minDoubleTimeIdx <<  std::endl;
        }
      }
      if(debugLevel >= 3){
        this->print(candidates);
      }
      if(this->debugLevel >= 1) std::cerr << "default time: " << timer.measure() << "[s]." << std::endl;
    }

    std::shared_ptr<FootStepCandidate> target = candidates[0];

    if(debugLevel >= 2) {
      std::cerr << "target " << target->supportLeg << std::endl;
      if(target->supportLeg != NUM_LEGS){
        std::cerr << target->supportLeg << " " << target->minTime << " " << target->p.transpose() << " " << target->theta << " " << target->minDoubleTimeIdx << std::endl;
      }
    }

    {
      if(target->supportLeg != NUM_LEGS){
        int supportLeg = (target->supportLeg);
        int swingLeg = (target->supportLeg == RLEG) ? LLEG : RLEG;
        std::vector<cnoid::Isometry3> path;
        std::vector<double> time;
        bool nearContact;
        this->calcPath(state, target, 0.0, legCoords, false, target->down, legCoords2D, legCoordsHorizontal,
                       path, time, nearContact);
        std::cerr << 0.0 << " " << legCoords[swingLeg].translation().transpose() << std::endl;
        for(int i=0;i<path.size();i++){
          std::cerr << time[i] << " " << path[i].translation().transpose() << std::endl;
        }
      }
    }


    // これを求める
    // std::vector<cnoid::Isometry3> refCoords(NUM_LEGS);
    // std::vector<double> refCoordsTime(NUM_LEGS);
    // std::vector<bool> refContact(NUM_LEGS);
    // cnoid::Isometry3 nextFootOriginCoords;
    // double nextFootOriginCoordsTime;


    output.qGoals.clear();
    if(nominal.nominalq.rows() == state.robot->numJoints()){
      output.qGoals.resize(1);
      output.qGoals[0].trajectory.resize(1);
      output.qGoals[0].trajectory[0].time = nominal.nominalqTime;
      output.qGoals[0].trajectory[0].q = nominal.nominalq;
      output.qGoals[0].trajectory[0].dq = cnoid::VectorX::Zero(state.robot->numJoints());
    }
    output.eeGoals.clear();
    output.contactGoals.clear();
    for(int leg=0;leg<NUM_LEGS;leg++){
      output.eeGoals.resize(output.eeGoals.size()+1);
      output.eeGoals.back().name = state.ee[leg].name;
      output.eeGoals.back().link = state.ee[leg].parentLink;
      output.eeGoals.back().localPose = state.ee[leg].localPose;
      output.eeGoals.back().frameLink = nullptr;
      output.eeGoals.back().framePose.setIdentity();
      for(int i=0;i<6;i++) output.eeGoals.back().freeAxis[i] = false;
      //output.eeGoals.back().priority = 1;
      output.eeGoals.back().priority = 0;

      bool contact = false;
      cnoid::Isometry3 contactPose = cnoid::Isometry3::Identity(); // world frame

      if(target->supportLeg == NUM_LEGS ||
         target->supportLeg == leg ||
         target->minDoubleTimeIdx > 0){
        output.eeGoals.back().trajectory.resize(1);
        output.eeGoals.back().trajectory[0].time = state.ee[leg].delayTimeOffset;
        output.eeGoals.back().trajectory[0].pose = legCoords[leg];
        contact = true;
        contactPose = legCoords[leg];
      }else{
        int supportLeg = (leg == RLEG) ? LLEG : RLEG;
        std::vector<cnoid::Isometry3> path; // world frame
        std::vector<double> time;
        this->calcPath(state, target, target->minTime, legCoords, false, target->down, legCoords2D, legCoordsHorizontal,
                       path, time, contact);
        contactPose = legCoordsHorizontal[target->supportLeg] * target->pose;

        if(this->debugLevel >= 2){
          std::cerr << 0.0 << " " << legCoords[leg].translation().transpose() << std::endl;
          for(int i=0;i<path.size();i++){
            std::cerr << time[i] << " " << path[i].translation().transpose() << std::endl;
          }
        }

        // delayTimeOffsetより手前を削除.
        if(time.back() < state.ee[leg].delayTimeOffset){
          path[0] = path.back(); path.resize(1);
          time[0] = time.back(); time.resize(1);
        }else{
          cnoid::Isometry3 p; // world frame. delayTimeOffset後の位置
          for(int i=0;i<path.size();i++){
            if(time[i] >= state.ee[leg].delayTimeOffset){
              cnoid::Isometry3 p0 = (i==0)? legCoords[leg] : path[i-1];
              double t0 = (i==0) ? 0.0 : time[i-1];
              cnoid::Isometry3 p1 = path[i];
              double t1 = time[i];
              p = mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{p0,p1},
                                          std::vector<double>{t1-state.ee[leg].delayTimeOffset,state.ee[leg].delayTimeOffset-t0});
              break;
            }
          }
          while(time[0] < state.ee[leg].delayTimeOffset){
            time.erase(time.begin());
            path.erase(path.begin());
          }
          time.insert(time.begin(), state.ee[leg].delayTimeOffset);
          path.insert(path.begin(), p);
        }
        output.eeGoals.back().trajectory.resize(path.size());
        double prevTime = 0.0;
        for(int i=0;i<path.size();i++){
          output.eeGoals.back().trajectory[i].time = time[i] - prevTime;
          prevTime = time[i];
          output.eeGoals.back().trajectory[i].pose = path[i];
        }
      }

      if(contact){
        output.contactGoals.resize(output.contactGoals.size()+1);
        output.contactGoals.back().name = state.ee[leg].name;
        output.contactGoals.back().link1 = state.ee[leg].parentLink;
        output.contactGoals.back().localPose1 = state.ee[leg].localPose;
        output.contactGoals.back().link2 = nullptr;
        for(int i=0;i<6;i++) output.contactGoals.back().freeAxis[i] = false;
        output.contactGoals.back().region = state.ee[leg].region;
        output.contactGoals.back().muTrans = state.ee[leg].muTrans;
        output.contactGoals.back().muRot = state.ee[leg].muRot;
        output.contactGoals.back().minFz = state.ee[leg].minFz;
        output.contactGoals.back().maxFz = state.ee[leg].maxFz;
        output.contactGoals.back().surface = state.ee[leg].hull;
        output.contactGoals.back().localPose2 = contactPose;
      }
    }
    for(int i=0;i<nominal.nominalEE.size();i++){
      output.eeGoals.resize(output.eeGoals.size()+1);
      output.eeGoals.back().name = nominal.nominalEE[i].name;
      output.eeGoals.back().link = nominal.nominalEE[i].link;
      output.eeGoals.back().localPose = nominal.nominalEE[i].localPose;
      output.eeGoals.back().frameLink = nominal.nominalEE[i].frameLink;
      output.eeGoals.back().framePose = nominal.nominalEE[i].framePose;
      output.eeGoals.back().freeAxis = nominal.nominalEE[i].freeAxis;
      output.eeGoals.back().priority = nominal.nominalEE[i].priority;

      if(nominal.nominalEE[i].frameLink != nullptr){
        output.eeGoals.back().trajectory.resize(1);
        output.eeGoals.back().trajectory[0].time = nominal.nominalEE[i].time;
        output.eeGoals.back().trajectory[0].pose = nominal.nominalEE[i].pose;
      }else{
        // foot origin相対
        output.eeGoals.back().trajectory.resize(1);
        double time;
        cnoid::Isometry3 footOrigin; // world系
        if(target->supportLeg == NUM_LEGS ||
           target->minDoubleTimeIdx > 0){
          time = 0.0;
          footOrigin = mathutil::orientCoordToAxis(mathutil::calcMidCoords(legCoords,std::vector<double>{0.5,0.5}),cnoid::Vector3::UnitZ());
        }else{
          time = target->minTime;
          int swingLeg = (target->supportLeg == RLEG) ? LLEG : RLEG;
          cnoid::Isometry3 targetPose = legCoordsHorizontal[target->supportLeg] * target->pose;// worldframe
          footOrigin = mathutil::orientCoordToAxis(mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{legCoords[target->supportLeg],targetPose},
                                                                            std::vector<double>{0.5,0.5}),cnoid::Vector3::UnitZ());
        }
        output.eeGoals.back().trajectory[0].time = std::max(time, nominal.nominalEE[i].time);
        output.eeGoals.back().trajectory[0].pose = footOrigin * nominal.nominalEE[i].pose;
      }
    }

    output.vrpGoals.clear();
    output.vrpGoals.resize(1);
    output.vrpGoals[0].omega = std::sqrt(state.g / nominal.nominalZ);
    if(target->supportLeg == NUM_LEGS) {
      output.vrpGoals[0].trajectory.resize(1);
      output.vrpGoals[0].trajectory[0].time = 0.0;
      cnoid::Vector3 p = mathutil::calcMidPos(std::vector<cnoid::Vector3>{legCoords[RLEG].translation(),legCoords[LLEG].translation()},
                                              std::vector<double>{0.5,0.5});
      p[2] += state.g / std::pow(std::sqrt(state.g / nominal.nominalZ),2);
      output.vrpGoals[0].trajectory[0].point = p;
    }else if(target->minDoubleTimeIdx > 0){
      output.vrpGoals[0].trajectory.resize(1);
      output.vrpGoals[0].trajectory[0].time = 0.0;
      cnoid::Vector3 p = legCoords[target->supportLeg].translation();
      p.head<2>() += legCoords2D[target->supportLeg].linear() * state.ee[target->supportLeg].copOffset;
      p[2] += state.g / std::pow(std::sqrt(state.g / nominal.nominalZ),2);
      output.vrpGoals[0].trajectory[0].point = p;
    }else{
      int swingLeg = (target->supportLeg == RLEG) ? LLEG : RLEG;
      cnoid::Isometry3 targetPose = legCoordsHorizontal[target->supportLeg] * target->pose; //world frame

      output.vrpGoals[0].trajectory.resize(3);
      output.vrpGoals[0].trajectory[0].time = 0.0;
      cnoid::Vector3 p = legCoords[target->supportLeg].translation();
      p.head<2>() += legCoords2D[target->supportLeg].linear() * state.ee[target->supportLeg].copOffset;
      p[2] += state.g / std::pow(std::sqrt(state.g / nominal.nominalZ),2);
      output.vrpGoals[0].trajectory[0].point = p;
      output.vrpGoals[0].trajectory[1].time = target->minTime;
      output.vrpGoals[0].trajectory[1].point = p;
      output.vrpGoals[0].trajectory[2].time = 0.0;
      cnoid::Vector3 p2 = targetPose.translation();
      p2.head<2>() += legCoords2D[target->supportLeg].linear() * target->pose2D.linear() * state.ee[swingLeg].copOffset;
      p2[2] += state.g / std::pow(std::sqrt(state.g / nominal.nominalZ),2);
      output.vrpGoals[0].trajectory[2].point = p2;
    }

    if(preferable) output.feasibility = Feasibility::FEASIBLE;
    else output.feasibility = Feasibility::UNPREFERABLE;

    this->prevTarget = target;
  }

  void FootStepGenerator::calcRealStrideLimitationHull(const int& swingLeg, const double& theta, const State& state, const std::vector<Eigen::Vector2d>& strideLimitationHull,
                                                       std::vector<Eigen::Vector2d>& realStrideLimitationHull) const {
    int supportLeg = (swingLeg == RLEG) ? LLEG : RLEG;
    cnoid::Matrix2d swingR = Eigen::Rotation2Dd(theta).toRotationMatrix();

    realStrideLimitationHull = strideLimitationHull;

    {
      // hullどうしのCollisionを考慮
      Eigen::Vector2d supportLegToSwingLeg = state.ee[swingLeg].defaultTranslatePos.head<2>() - state.ee[supportLeg].defaultTranslatePos.head<2>(); // 支持脚(水平)座標系.
      if(supportLegToSwingLeg.norm() > 0.0){
        Eigen::Vector2d supportLegToSwingLegDir = supportLegToSwingLeg.normalized();
        std::vector<Eigen::Vector2d> tmp;
        double supportLegHullSize = mathutil::findExtremes(state.ee[supportLeg].hull, supportLegToSwingLegDir, tmp);
        std::vector<Eigen::Vector2d> swingLegHull;
        for(int i=0;i<state.ee[swingLeg].hull.size();i++) swingLegHull.push_back(swingR * state.ee[swingLeg].hull[i]);
        double swingLegHullSize = mathutil::findExtremes(swingLegHull, - supportLegToSwingLegDir, tmp);
        double minDist = supportLegHullSize + state.ee[swingLeg].collisionMargin + swingLegHullSize;
        std::vector<Eigen::Vector2d> minDistHull; // supportLegToSwingLeg方向にminDistを満たすHull
        {
          Eigen::Isometry2d p;
          p.translation() = minDist * supportLegToSwingLegDir;
          p.linear() = (Eigen::Matrix2d() << supportLegToSwingLegDir[0], -supportLegToSwingLegDir[1],
                        supportLegToSwingLegDir[1], supportLegToSwingLegDir[0]).finished();
          minDistHull.push_back(p * Eigen::Vector2d(0.0, -1e2)); // 1e2はできるだけ大きな値. 大きすぎても桁落ちの危険があるので. とくに単精度浮動小数点を使う関数が混じっていると危険.
          minDistHull.push_back(p * Eigen::Vector2d(1e2, -1e2));
          minDistHull.push_back(p * Eigen::Vector2d(1e2, 1e2));
          minDistHull.push_back(p * Eigen::Vector2d(0.0, 1e2));
        }

        std::vector<Eigen::Vector2d> nextRealStrideLimitationHull = mathutil::calcIntersectConvexHull(realStrideLimitationHull, minDistHull);
        if(nextRealStrideLimitationHull.size() > 0) realStrideLimitationHull = nextRealStrideLimitationHull;
      }
    }

    {
      // swingLegから見てもsupportLegから見てもStrideLimitationHullの中にあることを確認 (swing中の干渉を防ぐ)
      const std::vector<Eigen::Vector2d>& strideLimitationHullFromSupport = realStrideLimitationHull;
      std::vector<Eigen::Vector2d> strideLimitationHullFromSwing(realStrideLimitationHull.size());
      for(int i=0;i<realStrideLimitationHull.size();i++){
        strideLimitationHullFromSwing[i] = swingR * realStrideLimitationHull[i];
      }
      std::vector<Eigen::Vector2d> nextRealStrideLimitationHull = mathutil::calcIntersectConvexHull(strideLimitationHullFromSupport, strideLimitationHullFromSwing);
      if(nextRealStrideLimitationHull.size() > 0) realStrideLimitationHull = nextRealStrideLimitationHull;

    }
  }

  inline bool FootStepGenerator::onTarget(const State& state, const std::shared_ptr<FootStepCandidate>& target, const std::vector<cnoid::Isometry3>& legCoords/*world frame*/, const std::vector<Eigen::Isometry2d>& legCoords2D, const std::vector<cnoid::Isometry3>& legCoordsHorizontal) const{
    int supportLeg = target->supportLeg;
    int swingLeg = (supportLeg == RLEG) ? LLEG : RLEG;
    cnoid::Isometry3 targetCoords = legCoordsHorizontal[supportLeg] * target->pose; // world frame
    Eigen::Isometry2d targetCoords2D = legCoords2D[supportLeg] * target->pose2D; // world frame
    double dtheta = std::abs(Eigen::Rotation2Dd(legCoords2D[swingLeg].linear().inverse() * targetCoords2D.linear()).smallestAngle());

    bool aboveTargetXY = true;
    if((targetCoords.translation().head<2>() - legCoords[swingLeg].translation().head<2>()).norm() > state.ee[swingLeg].liftXYThre2) aboveTargetXY = false;
    else if((targetCoords.translation().head<2>() - legCoords[swingLeg].translation().head<2>()).norm() <= state.ee[swingLeg].liftXYThre1) aboveTargetXY = true;
    else{
      if((targetCoords.translation().head<2>() - legCoords[swingLeg].translation().head<2>()).norm() <= (legCoords[swingLeg].translation()[2] - targetCoords.translation()[2]) * state.ee[swingLeg].liftRatioThre) aboveTargetXY = true;
      else aboveTargetXY = false;
    }
    bool aboveTargetTheta = true;
    if(dtheta > state.ee[swingLeg].liftThetaThre) aboveTargetTheta = false;
    else aboveTargetTheta = true;

    return aboveTargetXY && aboveTargetTheta;
  }

  inline void FootStepGenerator::calcPath(const State& state, const std::shared_ptr<FootStepCandidate>& target, double refTime, const std::vector<cnoid::Isometry3>& legCoords/*world frame*/, bool timeOnly, bool forceDown, const std::vector<Eigen::Isometry2d>& legCoords2D, const std::vector<cnoid::Isometry3>& legCoordsHorizontal,
                                   std::vector<cnoid::Isometry3>& path, std::vector<double>& time, bool& nearContact) const{
    path.clear();
    time.clear(); // time from start
    nearContact = false;

    /*
      XY
        t1: 上昇
          XYがtargetの周辺に無い or thetaがtargetの周辺に無い 場合のみ
        t2: 水平
        t3: 下降
      Theta
        t1 + t2: 旋回
        t3: goalへ

      constraints
        t1 > maxSwingLiftVelocity
        t2 > maxSwingXYVelocity
        t3 > maxSwingLandVelocity
        t1 + t2 > maxSwingThetaVelocity

      target:
        t1 + t2 + t3 = refTime
     */

    int supportLeg = target->supportLeg;
    int swingLeg = (supportLeg == RLEG) ? LLEG : RLEG;
    cnoid::Isometry3 targetCoords = legCoordsHorizontal[supportLeg] * target->pose; // world frame

    cnoid::Isometry3 p1 = cnoid::Isometry3::Identity();
    double t1 = 0.0;
    cnoid::Isometry3 p2 = cnoid::Isometry3::Identity();
    double t2 = 0.0;
    cnoid::Isometry3 p3 = cnoid::Isometry3::Identity();
    double t3 = 0.0;

    bool aboveTarget = this->onTarget(state, target, legCoords, legCoords2D, legCoordsHorizontal);
    if(forceDown) aboveTarget = true;

    // 並進成分を作成
    {
      if(!aboveTarget){
        double reqZ = legCoords[swingLeg].translation()[2];
        // double stepHeight = std::min((targetCoords.translation().head<2>() - legCoords[swingLeg].translation().head<2>()).norm() / state.ee[swingLeg].liftRatioThre,
        //                              state.ee[swingLeg].stepHeight);
        double stepHeight = state.ee[swingLeg].stepHeight;

        // goalの上空heightのZよりも低い位置にいるなら上昇
        reqZ = std::max(reqZ, targetCoords.translation()[2] + stepHeight);

        // 現在位置のheightmap+heightのZよりも低い位置にいるなら上昇
        reqZ = std::max(reqZ, legCoords[supportLeg].translation()[2] + stepHeight); // TODO heightmapから取得

        t1 = (reqZ - legCoords[swingLeg].translation()[2]) / state.ee[swingLeg].maxSwingLiftVelocity;
        p1.translation().head<2>() = legCoords[swingLeg].translation().head<2>();
        p1.translation()[2] = reqZ;
      }else{
        p1.translation() = legCoords[swingLeg].translation();
        t1 = 0.0;
      }
      // goal上空へ水平移動
      {
        Eigen::Vector2d reqXY = targetCoords.translation().head<2>();
        t2 = (reqXY - p1.translation().head<2>()).norm() / state.ee[swingLeg].maxSwingXYVelocity;
        p2.translation().head<2>() = reqXY;
        p2.translation()[2] = p1.translation()[2];
      }
      // goalへ上下移動
      {
        double reqZ = targetCoords.translation()[2] + state.ee[swingLeg].goalOffset;
        t3 = std::abs(reqZ - p2.translation()[2]) / state.ee[swingLeg].maxSwingLandVelocity;
        p3.translation().head<2>() = p2.translation().head<2>();
        p3.translation()[2] = reqZ;
      }
    }

    // 回転成分を付与
    {
      double dR = std::abs(cnoid::AngleAxisd(targetCoords.linear().inverse() * legCoords[swingLeg].linear()).angle());
      double reqTime = dR / state.ee[swingLeg].maxSwingThetaVelocity;
      if(t1 + t2 < reqTime){
        if(t1 + t2 == 0){
          t2 = reqTime;
        }else{
          double ratio = reqTime / (t1 + t2);
          t1 *= ratio;
          t2 *= ratio;
        }
      }
      if(!timeOnly){
        p1.linear() = mathutil::calcMidRot(std::vector<cnoid::Matrix3>{legCoords[swingLeg].linear(),targetCoords.linear()},
                                           std::vector<double>{t2,t1});
        p2.linear() = targetCoords.linear();
        p3.linear() = targetCoords.linear();
      }
    }

    // 延長してrefTimeに近づける.
    if(t1 + t2 + t3 < refTime) {
      if(t1 + t2 + t3 == 0) {
        t3 = refTime;
      }else{
        double ratio2 = refTime / (t1 + t2 + t3);
        t1 *= ratio2;
        t2 *= ratio2;
        t3 *= ratio2;
      }
    }


    if(aboveTarget &&
       legCoords[swingLeg].translation()[2] <= targetCoords.translation()[2] + state.ee[swingLeg].stepHeight) {
      nearContact = true;
    }

    double t = 0.0;
    if(t1 != 0) {
      t += t1;
      path.push_back(p1);
      time.push_back(t);
    }
    if(t2 != 0) {
      t += t2;
      path.push_back(p2);
      time.push_back(t);
    }
    t += t3;
    path.push_back(p3);
    time.push_back(t);

    // std::cerr << "path" << supportLeg << std::endl;
    // std::cerr << legCoords[swingLeg].translation().transpose() << std::endl;
    // for(int i=0;i<path.size();i++){
    //   std::cerr << time[i] << " " << path[i].translation().transpose() << " " << std::abs(cnoid::AngleAxisd(legCoords[swingLeg].linear().inverse() * path[i].linear()).angle()) << std::endl;
    // }

    return;
  }

  void FootStepGenerator::print(const std::vector<std::shared_ptr<FootStepCandidate> >& candidates) const{
    std::vector<std::vector<std::vector<double> > > data(NUM_LEGS); // swinglegがrleg/lleg. x. y.
    for(int swingLeg = 0;swingLeg<NUM_LEGS;swingLeg++){
      data[swingLeg].resize(this->sampledX[swingLeg].size());
      for(int x=0;x<this->sampledX[swingLeg].size();x++){
        data[swingLeg][x].resize(this->sampledY[swingLeg].size());
        for(int y=0;y<this->sampledY[swingLeg].size();y++){
          data[swingLeg][x][y] = 0.0;
        }
      }
    }

    for(int i=0;i<candidates.size();i++){
      if(candidates[i]->supportLeg == NUM_LEGS) continue;
      int swingLeg = candidates[i]->supportLeg == RLEG ? LLEG : RLEG;

      if(candidates[i]->maxTime > data[swingLeg][candidates[i]->xIdx][candidates[i]->yIdx]){
        data[swingLeg][candidates[i]->xIdx][candidates[i]->yIdx] = candidates[i]->maxTime;
      }
    }

    std::cerr << std::fixed << std::setprecision(2);
    for(int x=0;x<this->sampledX[RLEG].size() && x<this->sampledX[LLEG].size();x++){
      for(int y=0;y<this->sampledY[RLEG].size();y++){
        std::cerr << data[RLEG][x][y] << " ";
      }
      std::cerr << ": ";
      for(int y=0;y<this->sampledY[LLEG].size();y++){
        std::cerr << data[LLEG][x][y] << " ";
      }
      std::cerr << std::endl;
    }
    std::cerr << std::defaultfloat << std::setprecision(6);
  }
};
