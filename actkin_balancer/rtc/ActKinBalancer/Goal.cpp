#include "Goal.h"
#include "MathUtil.h"
#include <rtm_data_tools/rtm_data_tools.h>
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>

namespace actkin_balancer{
  // 返り値は水平
  bool RefRB::calcRBCoords(const State& state, cnoid::Isometry3& coords) {
    if(state.actContact[RLEG] && state.actContact[LLEG]) {
      coords = mathutil::orientCoordToAxis(mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{state.ee[RLEG].parentLink->T() * state.ee[RLEG].localPose,state.ee[LLEG].parentLink->T() * state.ee[LLEG].localPose},
                                                                   std::vector<double>{0.5,0.5}),
                                           cnoid::Vector3::UnitZ());
      return true;
    }else if(state.actContact[RLEG]){
      coords = mathutil::orientCoordToAxis(state.ee[RLEG].parentLink->T() * state.ee[RLEG].localPose,
                                           cnoid::Vector3::UnitZ());
      coords.translation() -= coords.linear() * state.ee[RLEG].defaultTranslatePos;
      return true;
    }else if(state.actContact[LLEG]){
      coords = mathutil::orientCoordToAxis(state.ee[LLEG].parentLink->T() * state.ee[LLEG].localPose,
                                           cnoid::Vector3::UnitZ());
      coords.translation() -= coords.linear() * state.ee[LLEG].defaultTranslatePos;
      return true;
    }else{
      return false;
    }
  }

  bool RefRB::isSatisfied(const State& state) const {
    cnoid::Isometry3 rbCoords;
    if(RefRB::calcRBCoords(state,rbCoords)){
      if((rbCoords.translation() - this->rb[0].translation()).head<2>().norm() <= this->xyGoalTorelance &&
         std::abs(cnoid::AngleAxisd(this->rb[0].linear() * rbCoords.linear().transpose()).angle()) <= this->yawGoalTorelance
         ) {
        return true;
      }
    }
    return false;
  }

  void Goal::init(const State& state){
    return;
  }

  void Goal::onStartBalancer(){
    this->rbGoals.clear();
    return;
  }


  void Goal::updateFromIdl(const State& state, const actkin_balancer_msgs::RefStateIdl& m_refState){
    this->updateFromIdl(state, m_refState.refRB);
  }

  void Goal::updateFromIdl(const State& state, const actkin_balancer_msgs::RefRBIdl& m_refRB){
    std::vector<std::shared_ptr<RefRB> > nextRBGoals;

    for(int i=0;i<1;i++){
      std::shared_ptr<RefRB> rbGoal;
      if(this->rbGoals.size() > 0){
        rbGoal = this->rbGoals[0];
      }else{
        rbGoal = std::make_shared<RefRB>();
      }

      rbGoal->rb.clear();
      for(int j=0;j<m_refRB.trajectory.length();j++){
        cnoid::Isometry3 goal; // global frame
        if(!rtm_data_tools::isAllFinite(m_refRB.trajectory[j].pose)){
          std::cerr << __FUNCTION__ << "rb not finite" << std::endl;
          continue;
        }
        eigen_rtm_conversions::poseRTMToEigen(m_refRB.trajectory[j].pose, goal);

        rbGoal->rb.push_back(mathutil::orientCoordToAxis(goal,cnoid::Vector3::UnitZ()));
      }

      if(rbGoal->rb.size() != 0) {
        nextRBGoals.push_back(rbGoal);
      }
    }

    std::swap(this->rbGoals, nextRBGoals);
  }

  void Goal::interpolate(const State& state, double dt) {
    while(this->rbGoals.size() > 0 && this->rbGoals[0]->rb.size() >= 2 && this->rbGoals[0]->isSatisfied(state)){
      this->rbGoals[0]->rb.erase(this->rbGoals[0]->rb.begin());
    }
  };
};
