#include "FootStepGenerator.h"
#include "MathUtil.h"
#include <cnoid/EigenUtil>

namespace actkin_balancer{
  void FootStepGenerator::calcFootSteps(const State& state, const Goal& goal, const std::string& instance_name, double dt,
                                        Output& output) const{

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
    // TODO

    std::vector<cnoid::Isometry3> legCoords{state.ee[RLEG].parentLink->T() * state.ee[RLEG].localPose, state.ee[LLEG].parentLink->T() * state.ee[LLEG].localPose};
    std::vector<cnoid::Isometry3> legCoordsHorizontal{
      mathutil::orientCoordToAxis(legCoords[RLEG], cnoid::Vector3::UnitZ()),
      mathutil::orientCoordToAxis(legCoords[LLEG], cnoid::Vector3::UnitZ())};
    std::vector<Eigen::Isometry2d> legCoords2D(NUM_LEGS);
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

    // 候補
    std::vector<FootStepCandidate> candidates;

    // stride limitationで初期化
    {
      if(state.actContact[RLEG] && state.actContact[LLEG]) {
        FootStepCandidate candidate;
        candidate.supportLeg = NUM_LEGS;
        candidates.push_back(candidate);
      }

      for(int supportLeg=0;supportLeg<NUM_LEGS;supportLeg++){
        int swingLeg = supportLeg == RLEG ? LLEG : RLEG;

        // thetaをsample
        std::vector<double> sampledThetas; // 支持脚相対
        {
          // resolutionThetaおきにsample
          for(double theta = state.ee[swingLeg].strideLimitationMinTheta;theta < state.ee[swingLeg].strideLimitationMaxTheta;theta += state.ee[swingLeg].resolutionTheta){
            sampledThetas.push_back(theta);
          }
          sampledThetas.push_back(state.ee[swingLeg].strideLimitationMaxTheta);
          // default位置をsample
          {
            double theta = defaultTheta[swingLeg];
            if(theta < state.ee[swingLeg].strideLimitationMaxTheta &&
               theta > state.ee[swingLeg].strideLimitationMinTheta){
              sampledThetas.push_back(theta);
            }
          }
        } // theta

        // XYをsample. Zには0を入れる
        std::vector<cnoid::Vector3> sampledPs; // 支持脚相対
        {
          // resolutionXYおきにsample
          std::vector<cnoid::Vector2d> tmp;
          double minX = - mathutil::findExtremes(state.ee[swingLeg].strideLimitationHull, -Eigen::Vector2d::UnitX(), tmp);
          double maxX = mathutil::findExtremes(state.ee[swingLeg].strideLimitationHull, Eigen::Vector2d::UnitX(), tmp);
          double minY = - mathutil::findExtremes(state.ee[swingLeg].strideLimitationHull, -Eigen::Vector2d::UnitY(), tmp);
          double maxY = mathutil::findExtremes(state.ee[swingLeg].strideLimitationHull, Eigen::Vector2d::UnitY(), tmp);
          for(double x=minX;;){
            for(double y=minY;;){
              sampledPs.push_back(cnoid::Vector3(x,y,0.0));
              if(y>=maxY) break;
              y = std::min(y+state.ee[swingLeg].resolutionXY, maxY);
            }
            if(x>=maxX) break;
            x = std::min(x+state.ee[swingLeg].resolutionXY, maxX);
          }
        }

        // stride limitationとZ高さのみ考慮してcandidateを作る
        for(int i=0;i<sampledThetas.size();i++){
          double theta = sampledThetas[i];
          std::vector<Eigen::Vector2d> realStrideLimitationHull; // 支持脚相対
          this->calcRealStrideLimitationHull(swingLeg, theta, state, state.ee[swingLeg].strideLimitationHull,
                                             realStrideLimitationHull);
          for(int j=0;j<sampledPs.size();j++){
            if(mathutil::isInsideHull(sampledPs[j].head<2>(),realStrideLimitationHull)){
              // 高さが範囲内かを判定する TODO
              FootStepCandidate candidate;
              candidate.supportLeg = supportLeg;
              candidate.keepDouble = false;
              candidate.p = sampledPs[j];
              candidate.theta = theta;
              candidate.minTime = 0.0;
              candidate.maxTime = state.ee[swingLeg].maxSwingTime;
              candidates.push_back(candidate);

              if(state.actContact[RLEG] && state.actContact[LLEG]){
                candidate.keepDouble = true;
                candidates.push_back(candidate);
              }
            }
          }
        }
      }
    }

    // Z情報を付与し、Z高さで絞り込み
    {
      std::vector<FootStepCandidate> nextCandidates;
      for(int i=0;i<candidates.size();i++){
        if(candidates[i].supportLeg == NUM_LEGS) {
          nextCandidates.push_back(candidates[i]);
          continue;
        }

        int swingLeg = candidates[i].supportLeg == RLEG ? LLEG : RLEG;

        double z = 0.0;
        // TODO. heightmapからzを計算.

        if(z <= state.ee[swingLeg].maxLandingHeight &&
           z >= state.ee[swingLeg].minLandingHeight){
          FootStepCandidate nextCandidate = candidates[i];
          nextCandidate.p[2] = z;
          nextCandidates.push_back(nextCandidate);
        }
      }

      if(nextCandidates.size() > 0.0) {
        candidates = nextCandidates;
      }else{
        std::cerr << "[" << instance_name << "] height not found" << std::endl;
      }
    }


    // reachable時間で絞り込み
    {
      std::vector<FootStepCandidate> nextCandidates;
      for(int i=0;i<candidates.size();i++){
        if(candidates[i].supportLeg == NUM_LEGS) {
          nextCandidates.push_back(candidates[i]);
          continue;
        }

        int supportLeg = candidates[i].supportLeg;
        int swingLeg = candidates[i].supportLeg == RLEG ? LLEG : RLEG;
        double nextMinTime = candidates[i].minTime;
        double nextMaxTime = candidates[i].maxTime;

        double minTimeTheta = std::abs(Eigen::Rotation2Dd(legCoords2D[supportLeg].linear().inverse() * legCoords2D[swingLeg].linear()).smallestAngle());
        if(minTimeTheta <= nextMinTime){
          // OK
        }else if(minTimeTheta <= nextMaxTime){
          nextMinTime = minTimeTheta;
        }else{
          continue;
        }

        std::vector<cnoid::Vector3> path;
        std::vector<double> time;
        this->calcPath(swingLeg, state, candidates[i].p,
                       path, time);
        double minTimeXY = time.back();

        if(minTimeXY <= nextMinTime){
          // OK
        }else if(minTimeXY <= nextMaxTime){
          nextMinTime = minTimeXY;
        }else{
          continue;
        }


        FootStepCandidate nextCandidate = candidates[i];
        nextCandidate.minTime = nextMinTime;
        nextCandidate.maxTime = nextMaxTime;
        nextCandidates.push_back(nextCandidate);
      }

      if(nextCandidates.size() > 0.0) {
        candidates = nextCandidates;
      }else{
        std::cerr << "[" << instance_name << "] Reachiable not found" << std::endl;
      }
    }

    // steppableで絞り込み
    {
      std::vector<FootStepCandidate> nextCandidates;
      for(int i=0;i<candidates.size();i++){
        if(candidates[i].supportLeg == NUM_LEGS) {
          nextCandidates.push_back(candidates[i]);
          continue;
        }

        // steppableかどうかを判定する TODO
        FootStepCandidate nextCandidate = candidates[i];
        nextCandidates.push_back(nextCandidate);

      }

      if(nextCandidates.size() > 0.0) {
        candidates = nextCandidates;
      }else{
        std::cerr << "[" << instance_name << "] steppable not found" << std::endl;
      }
    }

    // capturableで絞り込み
    {
      std::vector<FootStepCandidate> nextCandidates;
      for(int i=0;i<candidates.size();i++){
        if(candidates[i].supportLeg == NUM_LEGS ||
           candidates[i].keepDouble
           ) {
          // 現在の両足支持姿勢が0-step capturableか
          std::vector<Eigen::Vector2d> supportHull; // RLEG相対
          for(int v=0;v<state.ee[RLEG].safeHull.size();v++){
            supportHull.push_back(state.ee[RLEG].safeHull[v]);
          }
          for(int v=0;v<state.ee[LLEG].safeHull.size();v++){
            supportHull.push_back(legCoords2D[RLEG].inverse() * legCoords2D[LLEG] * state.ee[LLEG].safeHull[v]);
          }
          mathutil::calcConvexHull(supportHull,supportHull);
          Eigen::Vector2d dcm = legCoords2D[RLEG].inverse() * (state.robot->centerOfMass() + state.cogVel / nominal.omega).head<2>(); // RLEG相対
          if(!mathutil::isInsideHull(dcm, supportHull)) continue;

          nextCandidates.push_back(candidates[i]);
          continue;
        }

        // 着地時にcapturableかどうかを判定する
        int supportLeg = candidates[i].supportLeg;
        int swingLeg = candidates[i].supportLeg == RLEG ? LLEG : RLEG;
        Eigen::Vector2d dcm = legCoords2D[supportLeg].inverse() * (state.robot->centerOfMass() + state.cogVel / nominal.omega).head<2>(); // 支持脚
        std::vector<Eigen::Vector2d> endSupportHull; // 支持脚相対. 着時時
        for(int v=0;v<state.ee[supportLeg].safeHull.size();v++){
          endSupportHull.push_back(state.ee[supportLeg].safeHull[v]);
        }
        for(int v=0;v<state.ee[swingLeg].safeHull.size();v++){
          endSupportHull.push_back(legCoords2D[supportLeg].inverse() * legCoords2D[swingLeg] * state.ee[swingLeg].safeHull[v]);
        }
        mathutil::calcConvexHull(endSupportHull,endSupportHull);

        std::vector<double> capturableTime;
        for(double t = candidates[i].minTime;;){
          std::vector<Eigen::Vector2d> enddcm; // 支持脚相対. 着時時
          for(int v=0;v<state.ee[supportLeg].safeHull.size();v++){
            Eigen::Vector2d vrp = state.ee[supportLeg].safeHull[v];
            enddcm.push_back((dcm - vrp) * std::exp(nominal.omega * t) + vrp);
          }
          mathutil::calcConvexHull(enddcm,enddcm);

          std::vector<Eigen::Vector2d> intersect = mathutil::calcIntersectConvexHull(enddcm, endSupportHull);
          if(intersect.size() != 0) capturableTime.push_back(t);

          if(t>=candidates[i].maxTime) break;
          t = std::min(t+state.ee[swingLeg].resolutionTime, candidates[i].maxTime);
        }

        if(capturableTime.size() > 0){
          FootStepCandidate nextCandidate = candidates[i];
          nextCandidate.minTime = capturableTime.front();
          nextCandidate.maxTime = capturableTime.back();
          nextCandidates.push_back(nextCandidate);
        }
      }

      if(nextCandidates.size() > 0.0) {
        candidates = nextCandidates;
      }else{
        // この一歩ではcapture不可. CPの方向に最大限移動して、その中で可能な限り早く着地するのが良い.
        //   着地位置とswingLegの現在の位置との距離を見る
        double maxDistance = - std::numeric_limits<double>::max();
        std::vector<FootStepCandidate> bestCandidates;
        for(int i=0;i<candidates.size();i++){
          if(candidates[i].supportLeg == NUM_LEGS ||
             candidates[i].keepDouble) {
            continue;
          }

          int supportLeg = candidates[i].supportLeg;
          int swingLeg = candidates[i].supportLeg == RLEG ? LLEG : RLEG;
          Eigen::Vector2d dcm = legCoords2D[supportLeg].inverse() * (state.robot->centerOfMass() + state.cogVel / nominal.omega).head<2>(); // 支持脚相対
          Eigen::Vector2d dir = dcm.normalized(); // capturbleでないのでnormは必ず!=0
          Eigen::Vector2d swingp = legCoords2D[supportLeg].inverse() * legCoords2D[swingLeg].translation(); //支持脚相対
          double distance = (candidates[i].p.head<2>() - swingp).dot(dir);

          if(distance > maxDistance + 0.001/*epsilon*/){
            maxDistance = distance;
            bestCandidates.clear();
            FootStepCandidate nextCandidate = candidates[i];
            nextCandidate.maxTime = nextCandidate.minTime;
            bestCandidates.push_back(nextCandidate);
          }else if(distance > maxDistance - 0.001/*epsilon*/){
            FootStepCandidate nextCandidate = candidates[i];
            nextCandidate.maxTime = nextCandidate.minTime;
            bestCandidates.push_back(nextCandidate);
          }
        }

        double minTime = std::numeric_limits<double>::max();
        for(int i=0;i<bestCandidates.size();i++){
          if(bestCandidates[i].minTime < minTime - 0.001/*epsilon*/){
            minTime = bestCandidates[i].minTime;
            nextCandidates.clear();
            nextCandidates.push_back(bestCandidates[i]);
          }else if(bestCandidates[i].minTime < minTime + 0.001/*epsilon*/){
            nextCandidates.push_back(bestCandidates[i]);
          }
        }

        if(nextCandidates.size() > 0.0) {
          candidates = nextCandidates;
        }else{
          std::cerr << "[" << instance_name << "] capturable not found" << std::endl;
        }
      }
    }


    // default stepで絞り込み.
    // thetaがcurerntよりもどれだけ近づくか
    {
      double maxDistance = - std::numeric_limits<double>::max(); // 近づいた距離
      std::vector<FootStepCandidate> nextCandidates;
      for(int i=0;i<candidates.size();i++){
        if(candidates[i].supportLeg == NUM_LEGS) {
          if(goal.rbGoals.size() == 0 ||
             (goal.rbGoals.size() == 1 && goal.rbGoals[0]->isSatisfied(state))){
            nextCandidates.clear();
            nextCandidates.push_back(candidates[i]);
            break;
          }

          double distance = 0.0;
          if(distance < maxDistance + 0.001/*epsilon*/) {
            maxDistance = distance;
            nextCandidates.clear();
            nextCandidates.push_back(candidates[i]);
          }else if(distance < maxDistance - 0.001/*epsilon*/) {
            nextCandidates.push_back(candidates[i]);
          }

          continue;
        }

        int supportLeg = candidates[i].supportLeg;
        int swingLeg = candidates[i].supportLeg == RLEG ? LLEG : RLEG;
        double currentTheta = Eigen::Rotation2Dd(legCoords2D[supportLeg].linear().inverse() * legCoords2D[swingLeg].linear()).smallestAngle(); // 支持脚相対
        double currentError = std::abs(currentTheta - defaultTheta[swingLeg]);
        double endTheta = candidates[i].theta; // 支持脚相対
        double endError = std::abs(endTheta - defaultTheta[swingLeg]);
        double distance = currentError - endError;

        if(distance > maxDistance + 0.001/*epsilon*/){
          maxDistance = distance;
          nextCandidates.clear();
          nextCandidates.push_back(candidates[i]);
        }else if(distance > maxDistance - 0.001/*epsilon*/){
          nextCandidates.push_back(candidates[i]);
        }
      }

      if(nextCandidates.size() > 0.0) {
        candidates = nextCandidates;
      }else{
        std::cerr << "[" << instance_name << "] default theta not found" << std::endl;
      }
    }

    // default stepで絞り込み.
    // default velocityの時間との誤差
    {
      double minError = std::numeric_limits<double>::max(); // defaultに要する時間で割った比
      std::vector<FootStepCandidate> nextCandidates;
      for(int i=0;i<candidates.size();i++){
        if(candidates[i].supportLeg == NUM_LEGS) {
          nextCandidates.clear();
          nextCandidates.push_back(candidates[i]);
          break;
        }

        int swingLeg = candidates[i].supportLeg == RLEG ? LLEG : RLEG;

        std::vector<cnoid::Vector3> path;
        std::vector<double> time;
        this->calcPath(swingLeg, state, candidates[i].p,
                       path, time);
        double defaultTime = time.back() / state.ee[swingLeg].defaultSwingVelocityRatio;

        FootStepCandidate nextCandidate = candidates[i];
        double error;
        if(defaultTime < nextCandidate.minTime) {
          error = nextCandidate.minTime - defaultTime;
          nextCandidate.maxTime = nextCandidate.minTime;
        }else if(defaultTime > nextCandidate.maxTime) {
          error = defaultTime - nextCandidate.maxTime;
          nextCandidate.minTime = nextCandidate.maxTime;
        }else{
          error = 0.0;
          nextCandidate.minTime = nextCandidate.maxTime = defaultTime;
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

    FootStepCandidate target = candidates[0];


    // これを求める
    // std::vector<cnoid::Isometry3> refCoords(NUM_LEGS);
    // std::vector<double> refCoordsTime(NUM_LEGS);
    // std::vector<bool> refContact(NUM_LEGS);
    // cnoid::Isometry3 nextFootOriginCoords;
    // double nextFootOriginCoordsTime;


    // 共通設定
    output.qGoals.clear();
    if(nominal.nominalq.rows() == state.robot->numJoints()){
      output.qGoals.resize(1);
      output.qGoals[0].trajectory.resize(1);
      output.qGoals[0].trajectory[0].time = nominal.time;
      output.qGoals[0].trajectory[0].q = nominal.nominalq;
      output.qGoals[0].trajectory[0].dq = cnoid::VectorX::Zero(state.robot->numJoints());
    }
    output.eeGoals.clear();
    for(int i=0;i<nominal.nominalEE.size();i++){
      output.eeGoals.resize(output.eeGoals.size()+1);
      output.eeGoals[i].name = nominal.nominalEE[i].name;
      output.eeGoals[i].link = nominal.nominalEE[i].link;
      output.eeGoals[i].localPose = nominal.nominalEE[i].localPose;
      output.eeGoals[i].frameLink = nominal.nominalEE[i].frameLink;
      output.eeGoals[i].framePose = nominal.nominalEE[i].framePose;
      output.eeGoals[i].trajectory.resize(1);
      output.eeGoals[i].trajectory[0].time = nominal.nominalEE[i].time; // TODO foot origin相対
      output.eeGoals[i].trajectory[0].pose = nominal.nominalEE[i].pose; // TODO foot origin相対
      output.eeGoals[i].freeAxis = nominal.nominalEE[i].freeAxis;
      output.eeGoals[i].priority = nominal.nominalEE[i].priority;
    }

  }

  void FootStepGenerator::calcDefaultStep(int swingLeg, const State& state, const Goal& goal,
                                          cnoid::Isometry3& landingCoords){
    int supportLeg = swingLeg==RLEG ? LLEG : RLEG;
    cnoid::Isometry3 supportCoords = mathutil::orientCoordToAxis(state.ee[supportLeg].parentLink->T() * state.ee[supportLeg].localPose,
                                                                 cnoid::Vector3::UnitZ()); //world frame
    cnoid::Isometry3 supportMidCoords; //world frame
    supportMidCoords.translation() = supportCoords * (- state.ee[supportLeg].defaultTranslatePos);
    supportMidCoords.linear() = supportCoords.linear();

    cnoid::Isometry3 trans = supportMidCoords.inverse() * goal.rbGoals[0]->rb[0]; // supportMidCoords frame
    cnoid::Vector3 diff; // supportMidCoords frame
    diff.head<2>() = trans.translation().head<2>();
    diff[2] = cnoid::rpyFromRot(trans.linear())[2];
    diff[2] = std::min(state.ee[swingLeg].strideLimitationMaxTheta,
                       std::max(state.ee[swingLeg].strideLimitationMinTheta,
                                diff[2]));
    std::vector<cnoid::Vector3> strideLimitationHull;

  }


  void FootStepGenerator::calcRealStrideLimitationHull(const int& swingLeg, const double& theta, const State& state, const std::vector<Eigen::Vector2d>& strideLimitationHull,
                                                       std::vector<Eigen::Vector2d>& realStrideLimitationHull) const {
    int supportLeg = swingLeg == RLEG ? LLEG : RLEG;
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
          minDistHull.push_back(p * Eigen::Vector2d(0.0, -1e10));
          minDistHull.push_back(p * Eigen::Vector2d(1e10, -1e10));
          minDistHull.push_back(p * Eigen::Vector2d(1e10, 1e10));
          minDistHull.push_back(p * Eigen::Vector2d(0.0, 1e10));
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


  void FootStepGenerator::calcPath(int swingLeg, const State& state, const cnoid::Vector3& target,
                                   std::vector<cnoid::Vector3>& path, std::vector<double>& time) const{
    path.clear();
    time.clear(); // time from start

    std::vector<cnoid::Isometry3> legCoords{state.ee[RLEG].parentLink->T() * state.ee[RLEG].localPose, state.ee[LLEG].parentLink->T() * state.ee[LLEG].localPose};

    cnoid::Vector3 p = legCoords[swingLeg].translation();
    double t = 0.0;
    if((target.head<2>() - legCoords[swingLeg].translation().head<2>()).norm() > state.ee[swingLeg].liftThre){
      // goalの上空heightのZよりも低い位置にいるなら上昇
      {
        double reqZ = target[2] + state.ee[swingLeg].stepHeight;
        if(p[2] < reqZ){
          t += (reqZ - p[2]) / state.ee[swingLeg].maxSwingLiftVelocity;
          p[2] = reqZ;

          path.push_back(p);
          time.push_back(t);
        }
      }
      // 現在位置のheightmap+heightのZよりも低い位置にいるなら上昇
      {
        double reqZ = 0.0 + state.ee[swingLeg].stepHeight; // TODO heightmapから取得
        if(p[2] < reqZ){
          t += (reqZ - p[2]) / state.ee[swingLeg].maxSwingLiftVelocity;
          p[2] = reqZ;

          path.push_back(p);
          time.push_back(t);
        }
      }
    }
    // goal上空へ水平移動
    {
      Eigen::Vector2d reqXY = target.head<2>();
      t += (reqXY - p.head<2>()).norm() / state.ee[swingLeg].maxSwingXYVelocity;
      p.head<2>() = reqXY;

      path.push_back(p);
      time.push_back(t);
    }
    // goalへ上下移動
    {
      double reqZ = target[2] + state.ee[swingLeg].goalOffset;
      t += std::abs(reqZ - p[2]) / state.ee[swingLeg].maxSwingLandVelocity;
      p[2] = reqZ;

      path.push_back(p);
      time.push_back(t);
    }
  }
};
