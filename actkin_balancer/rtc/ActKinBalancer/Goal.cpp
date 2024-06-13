#include "Goal.h"

namespace actkin_balancer{
  bool Goal::calcRBCoords(const State& state, cnoid::Isometry3& coords) {
    if(state.actContacts[RLEG] && state.actContacts[LLEG]) {
      coords = mathutil::orientCoordToAxis(mathutil::calcMidCoords(std::vector<cnoid::Isometry3>{state.ee[RLEG].parentLink->T() * state.ee[RLEG].localPose,state.ee[LLEG].parentLink->T() * state.ee[LLEG].localPose},
                                                                   std::vector<double>{0.5,0.5}),
                                           cnoid::Vector3::UnitZ());
      return true;
    }else if(state.actContacts[RLEG]){
      coords = mathutil::orientCoordToAxis(state.ee[RLEG].parentLink->T() * state.ee[RLEG].localPose,
                                           cnoid::Vector3::UnitZ());
      return true;
    }else if(state.actContacts[LLEG]){
      coords = mathutil::orientCoordToAxis(state.ee[LLEG].parentLink->T() * state.ee[LLEG].localPose,
                                           cnoid::Vector3::UnitZ());
      return true;
    }else{
      return false;
    }
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

      cnoid::Vector3 p; // world frame
      cnoid::Vector3 dp; // world frame
      cnoid::Vector3 ddp; // world frame
      if(rbGoal->rb.size()>0){
        rbGoal->rb[0].value(p,dp,ddp);
      }else{ // 今回始めて現れたrb
        p = state.robot->centerOfMass(); // ここから
        dp = state.cogVel;//state.cogVel.value();
        ddp.setZero();
      }

      rbGoal->rb.clear();
      for(int j=0;j<m_refRB.trajectory.length();j++){
        double time;
        cnoid::Vector3 goal_p; // global frame
        if(!std::isfinite(m_refRB.trajectory[j].time)){
          std::cerr << __FUNCTION__ << " time is not finite!" << std::endl;
          continue;
        }
        time = std::max(0.0, m_refRB.trajectory[j].time);
        if(!rtm_data_tools::isAllFinite(m_refRB.trajectory[j].point)){
          std::cerr << __FUNCTION__ << "point not finite" << std::endl;
          continue;
        }
        eigen_rtm_conversions::pointRTMToEigen(m_refRB.trajectory[j].point, goal_p);

        rbGoal->rb.emplace_back(p,dp,ddp,cpp_filters::LINEAR);
        rbGoal->rb.back().setGoal(goal_p, time);

        p = goal_p;
        dp = cnoid::Vector3::Zero();
        ddp = cnoid::Vector3::Zero();
      }

      if(rbGoal->rb.size() == 0) {
        std::cerr << __FUNCTION__ << "trajectory is empty" << std::endl;
        continue;
      }

      nextRBGoals.push_back(rbGoal);
    }

    std::swap(this->rbGoals, nextRBGoals);
  }

};
