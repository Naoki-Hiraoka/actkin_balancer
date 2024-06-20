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
    bool preferable = true;
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
        cnoid::Isometry3 targetPose; // world frame
        targetPose.translation() = legCoordsHorizontal[supportLeg] * candidates[i].p;
        targetPose.linear() = legCoordsHorizontal[supportLeg].linear() * cnoid::AngleAxisd(candidates[i].theta, cnoid::Vector3::UnitZ());

        double nextMinTime = candidates[i].minTime;
        double nextMaxTime = candidates[i].maxTime;

        std::vector<cnoid::Isometry3> path;
        std::vector<double> time;
        bool nearContact;
        this->calcPath(swingLeg, state, targetPose, 0.0,
                       path, time, nearContact);
        double minTime = time.back();

        if(minTime <= nextMinTime){
          // OK
        }else if(minTime <= nextMaxTime){
          nextMinTime = minTime;
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
          Eigen::Vector2d dcm = legCoords2D[RLEG].inverse() * (state.robot->centerOfMass() + state.cogVel / std::sqrt(state.g / nominal.nominalZ)).head<2>(); // RLEG相対
          if(!mathutil::isInsideHull(dcm, supportHull)) continue;

          nextCandidates.push_back(candidates[i]);
          continue;
        }

        // 着地時にcapturableかどうかを判定する
        int supportLeg = candidates[i].supportLeg;
        int swingLeg = candidates[i].supportLeg == RLEG ? LLEG : RLEG;
        Eigen::Vector2d dcm = legCoords2D[supportLeg].inverse() * (state.robot->centerOfMass() + state.cogVel / std::sqrt(state.g / nominal.nominalZ)).head<2>(); // 支持脚
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
            enddcm.push_back((dcm - vrp) * std::exp(std::sqrt(state.g / nominal.nominalZ) * t) + vrp);
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
          Eigen::Vector2d dcm = legCoords2D[supportLeg].inverse() * (state.robot->centerOfMass() + state.cogVel / std::sqrt(state.g / nominal.nominalZ)).head<2>(); // 支持脚相対
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

        int supportLeg = candidates[i].supportLeg;
        int swingLeg = candidates[i].supportLeg == RLEG ? LLEG : RLEG;
        cnoid::Isometry3 targetPose; // world frame
        targetPose.translation() = legCoordsHorizontal[supportLeg] * candidates[i].p;
        targetPose.linear() = legCoordsHorizontal[supportLeg].linear() * cnoid::AngleAxisd(candidates[i].theta, cnoid::Vector3::UnitZ());

        std::vector<cnoid::Isometry3> path;
        std::vector<double> time;
        bool nearContact;
        this->calcPath(swingLeg, state, targetPose, 0.0,
                       path, time, nearContact);
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
      output.eeGoals.back().priority = 1;

      bool contact = false;
      cnoid::Isometry3 contactPose = cnoid::Isometry3::Identity();

      if(target.supportLeg == NUM_LEGS ||
         target.supportLeg == leg ||
         target.keepDouble){
        output.eeGoals.back().trajectory.resize(1);
        output.eeGoals.back().trajectory[0].time = 0.0;
        output.eeGoals.back().trajectory[0].pose = legCoords[leg];
        contact = true;
        contactPose = legCoords[leg];
      }else{
        int supportLeg = leg == RLEG ? LLEG : RLEG;
        cnoid::Isometry3 targetPose; // world frame
        targetPose.translation() = legCoordsHorizontal[target.supportLeg] * target.p;
        targetPose.linear() = legCoordsHorizontal[target.supportLeg].linear() * cnoid::AngleAxisd(target.theta, cnoid::Vector3::UnitZ());

        std::vector<cnoid::Isometry3> path; // world frame
        std::vector<double> time;
        this->calcPath(leg, state, targetPose, target.minTime,
                       path, time, contact);
        contactPose = targetPose;

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
        for(int i=0;i<path.size();i++){
          output.eeGoals.back().trajectory[i].time = time[i];
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
        output.contactGoals.back().wrenchC = state.ee[leg].wrenchC;
        output.contactGoals.back().wrenchld = state.ee[leg].wrenchld;
        output.contactGoals.back().wrenchud = state.ee[leg].wrenchud;
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
        if(target.supportLeg == NUM_LEGS ||
           target.keepDouble){
          time = 0.0;
          footOrigin = mathutil::orientCoordToAxis(mathutil::calcMidCoords(legCoords,std::vector<double>{0.5,0.5}),cnoid::Vector3::UnitZ());
        }else{
          time = target.minTime;
          int swingLeg = target.supportLeg == RLEG ? LLEG : RLEG;
          cnoid::Isometry3 targetPose;
          targetPose.translation() = legCoordsHorizontal[target.supportLeg] * target.p;
          targetPose.linear() = legCoordsHorizontal[target.supportLeg].linear() * cnoid::AngleAxisd(target.theta, cnoid::Vector3::UnitZ());
          footOrigin = mathutil::orientCoordToAxis(mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{legCoords[target.supportLeg],targetPose},
                                                                            std::vector<double>{0.5,0.5}),cnoid::Vector3::UnitZ());
        }
        output.eeGoals.back().trajectory[0].time = time + nominal.nominalEE[i].time;
        output.eeGoals.back().trajectory[0].pose = footOrigin * nominal.nominalEE[i].pose;
      }
    }

    output.vrpGoals.clear();
    output.vrpGoals.resize(1);
    output.vrpGoals[0].omega = std::sqrt(state.g / nominal.nominalZ);
    if(target.supportLeg == NUM_LEGS) {
      output.vrpGoals[0].trajectory.resize(1);
      output.vrpGoals[0].trajectory[0].time = 0.0;
      cnoid::Vector3 p = mathutil::calcMidPos(std::vector<cnoid::Vector3>{legCoords[RLEG].translation(),legCoords[LLEG].translation()},
                                              std::vector<double>{0.5,0.5});
      p[2] += state.g / std::pow(std::sqrt(state.g / nominal.nominalZ),2);
      output.vrpGoals[0].trajectory[0].point = p;
    }else if(target.keepDouble){
      output.vrpGoals[0].trajectory.resize(1);
      output.vrpGoals[0].trajectory[0].time = 0.0;
      cnoid::Vector3 p = legCoords[target.supportLeg].translation();
      p[2] += state.g / std::pow(std::sqrt(state.g / nominal.nominalZ),2);
      output.vrpGoals[0].trajectory[0].point = p;
    }else{
      int swingLeg = target.supportLeg == RLEG ? LLEG : RLEG;
      cnoid::Isometry3 targetPose;
      targetPose.translation() = legCoordsHorizontal[target.supportLeg] * target.p;
      targetPose.linear() = legCoordsHorizontal[target.supportLeg].linear() * cnoid::AngleAxisd(target.theta, cnoid::Vector3::UnitZ());

      output.vrpGoals[0].trajectory.resize(3);
      output.vrpGoals[0].trajectory[0].time = 0.0;
      cnoid::Vector3 p = legCoords[target.supportLeg].translation();
      p[2] += state.g / std::pow(std::sqrt(state.g / nominal.nominalZ),2);
      output.vrpGoals[0].trajectory[0].point = p;
      output.vrpGoals[0].trajectory[1].time = target.minTime;
      output.vrpGoals[0].trajectory[1].point = p;
      output.vrpGoals[0].trajectory[2].time = target.minTime;
      cnoid::Vector3 p2 = targetPose.translation();
      p2[2] += state.g / std::pow(std::sqrt(state.g / nominal.nominalZ),2);
      output.vrpGoals[0].trajectory[2].point = p;
    }

    if(preferable) output.feasibility = Feasibility::FEASIBLE;
    else output.feasibility = Feasibility::UNPREFERABLE;


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


  void FootStepGenerator::calcPath(int swingLeg, const State& state, const cnoid::Isometry3& target/*world frame*/, double refTime,
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

    std::vector<cnoid::Isometry3> legCoords{state.ee[RLEG].parentLink->T() * state.ee[RLEG].localPose, state.ee[LLEG].parentLink->T() * state.ee[LLEG].localPose};

    cnoid::Isometry3 p1 = cnoid::Isometry3::Identity();
    double t1 = 0.0;
    cnoid::Isometry3 p2 = cnoid::Isometry3::Identity();
    double t2 = 0.0;
    cnoid::Isometry3 p3 = cnoid::Isometry3::Identity();
    double t3 = 0.0;

    // 並進成分を作成
    {
      bool aboveTargetXY = true;
      if((target.translation().head<2>() - legCoords[swingLeg].translation().head<2>()).norm() > state.ee[swingLeg].liftXYThre2) aboveTargetXY = false;
      else if((target.translation().head<2>() - legCoords[swingLeg].translation().head<2>()).norm() <= state.ee[swingLeg].liftXYThre1) aboveTargetXY = true;
      else{
        if((target.translation().head<2>() - legCoords[swingLeg].translation().head<2>()).norm() <= (legCoords[swingLeg].translation()[2] - target.translation()[2]) * state.ee[swingLeg].liftRatioThre) aboveTargetXY = true;
        else aboveTargetXY = false;
      }
      bool aboveTargetTheta = true;
      if(std::abs(cnoid::AngleAxisd(target.linear().inverse() * legCoords[swingLeg].linear()).angle()) > state.ee[swingLeg].liftThetaThre) aboveTargetTheta = false;
      else aboveTargetTheta = true;

      if(!aboveTargetXY || !aboveTargetTheta){
        double reqZ = legCoords[swingLeg].translation()[2];

        // goalの上空heightのZよりも低い位置にいるなら上昇
        reqZ = std::max(reqZ, target.translation()[2] + state.ee[swingLeg].stepHeight);

        // 現在位置のheightmap+heightのZよりも低い位置にいるなら上昇
        reqZ = std::max(reqZ, 0.0 + state.ee[swingLeg].stepHeight); // TODO heightmapから取得

        t1 = (reqZ - legCoords[swingLeg].translation()[2]) / state.ee[swingLeg].maxSwingLiftVelocity;
        p1.translation().head<2>() = legCoords[swingLeg].translation().head<2>();
        p1.translation()[2] = reqZ;
      }else{
        p1.translation() = legCoords[swingLeg].translation();
        t1 = 0.0;
      }
      // goal上空へ水平移動
      {
        Eigen::Vector2d reqXY = target.translation().head<2>();
        t2 = (reqXY - p1.translation().head<2>()).norm() / state.ee[swingLeg].maxSwingXYVelocity;
        p2.translation().head<2>() = reqXY;
        p2.translation()[2] = p1.translation()[2];
      }
      // goalへ上下移動
      {
        double reqZ = target.translation()[2] + state.ee[swingLeg].goalOffset;
        t3 = std::abs(reqZ - p2.translation()[2]) / state.ee[swingLeg].maxSwingLandVelocity;
        p3.translation().head<2>() = p2.translation().head<2>();
        p3.translation()[2] = reqZ;
      }
    }

    // 回転成分を付与
    double ratio = 1.0;
    {
      double reqTime = std::abs(cnoid::AngleAxisd(target.linear().inverse() * legCoords[swingLeg].linear()).angle()) / state.ee[swingLeg].maxSwingThetaVelocity;
      if(t1 + t2 < reqTime){
        if(t1 + t2 == 0){
          t2 = reqTime;
          p2.linear() = target.linear();
          p3.linear() = target.linear();
        }else{
          ratio = reqTime / (t1 + t2);
          t1 *= ratio;
          t2 *= ratio;
          p1 = mathutil::calcMidRot(std::vector<cnoid::Matrix3>{legCoords[swingLeg].linear(),target.linear()},
                                    std::vector<double>{t1,t2});
          p2.linear() = target.linear();
          p3.linear() = target.linear();
        }
      }
    }

    // 延長してrefTimeに近づける.
    if(t1 + t2 + t3 < refTime) {
      // ratioぶんt3を延長する
      if(t1 + t2 + t3 * ratio < refTime) t3 *= ratio;
      else t3 = refTime - t1 - t2;

      // 全体を延長する
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
    }


    if(t1 == 0 &&
       legCoords[swingLeg].translation()[2] <= target.translation()[2] + state.ee[swingLeg].stepHeight) {
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

    return;
  }
};
