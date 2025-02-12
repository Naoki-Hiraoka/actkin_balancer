#include "Output.h"
#include <eigen_rtm_conversions/eigen_rtm_conversions.h>

namespace actkin_balancer {

  void Output::convertToIdl(const State& state, actkin_stabilizer_msgs::RefStateIdl& m_refState) const{
    m_refState.refEEPose.length(this->eeGoals.size());
    for(int i=0;i<this->eeGoals.size();i++){
      m_refState.refEEPose[i].name = this->eeGoals[i].name.c_str();
      m_refState.refEEPose[i].link = state.linkNameMap.find(this->eeGoals[i].link)->second.c_str();
      eigen_rtm_conversions::poseEigenToRTM(this->eeGoals[i].localPose, m_refState.refEEPose[i].localPose);
      m_refState.refEEPose[i].frameId = this->eeGoals[i].frameLink ? state.linkNameMap.find(this->eeGoals[i].frameLink)->second.c_str() : "";
      eigen_rtm_conversions::poseEigenToRTM(this->eeGoals[i].framePose, m_refState.refEEPose[i].framePose);
      m_refState.refEEPose[i].freeAxis.length(6); // 自動でサイズ6にならないみたい
      for(int j=0;j<6;j++) m_refState.refEEPose[i].freeAxis[j] = this->eeGoals[i].freeAxis[j];
      m_refState.refEEPose[i].priority = this->eeGoals[i].priority;

      m_refState.refEEPose[i].trajectory.length(this->eeGoals[i].trajectory.size());
      for(int j=0;j<this->eeGoals[i].trajectory.size();j++){
        m_refState.refEEPose[i].trajectory[j].time = this->eeGoals[i].trajectory[j].time;
        eigen_rtm_conversions::poseEigenToRTM(this->eeGoals[i].trajectory[j].pose,m_refState.refEEPose[i].trajectory[j].pose);
        eigen_rtm_conversions::velocityEigenToRTM(this->eeGoals[i].trajectory[j].velocity,m_refState.refEEPose[i].trajectory[j].velocity);
        eigen_rtm_conversions::vectorEigenToRTM(this->eeGoals[i].trajectory[j].wrench,m_refState.refEEPose[i].trajectory[j].wrench);
      }
    }

    if(this->vrpGoals.size() == 0){
      m_refState.refVRP.omega = 0;
      m_refState.refVRP.trajectory.length(0);
    }else{
      m_refState.refVRP.omega = this->vrpGoals[0].omega;
      m_refState.refVRP.trajectory.length(this->vrpGoals[0].trajectory.size());
      for(int i=0;i<this->vrpGoals[0].trajectory.size();i++){
        m_refState.refVRP.trajectory[i].time = this->vrpGoals[0].trajectory[i].time;
        eigen_rtm_conversions::pointEigenToRTM(this->vrpGoals[0].trajectory[i].point, m_refState.refVRP.trajectory[i].point);
      }
    }

    m_refState.refq.trajectory.length(this->qGoals.size());
    for(int i=0;i<this->qGoals.size();i++){
      m_refState.refq.trajectory[i].time = this->qGoals[0].trajectory[i].time;
      eigen_rtm_conversions::vectorEigenToRTM(this->qGoals[0].trajectory[i].q, m_refState.refq.trajectory[i].q);
      eigen_rtm_conversions::vectorEigenToRTM(this->qGoals[0].trajectory[i].dq, m_refState.refq.trajectory[i].dq);
    }

    m_refState.refContact.length(this->contactGoals.size());
    for(int i=0;i<this->contactGoals.size();i++){
      m_refState.refContact[i].name = this->contactGoals[i].name.c_str();
      m_refState.refContact[i].link1 = this->contactGoals[i].link1 ? state.linkNameMap.find(this->contactGoals[i].link1)->second.c_str() : "";
      eigen_rtm_conversions::poseEigenToRTM(this->contactGoals[i].localPose1, m_refState.refContact[i].localPose1);
      m_refState.refContact[i].link2 = this->contactGoals[i].link2 ? state.linkNameMap.find(this->contactGoals[i].link2)->second.c_str() : "";
      m_refState.refContact[i].freeAxis.length(6);
      for(int j=0;j<6;j++) m_refState.refContact[i].freeAxis[j] = this->contactGoals[i].freeAxis[j];
      eigen_rtm_conversions::matrixEigenToRTM(this->contactGoals[i].region.C, m_refState.refContact[i].region.C);
      eigen_rtm_conversions::vectorEigenToRTM(this->contactGoals[i].region.ld, m_refState.refContact[i].region.ld);
      eigen_rtm_conversions::vectorEigenToRTM(this->contactGoals[i].region.ud, m_refState.refContact[i].region.ud);
      m_refState.refContact[i].muTrans = this->contactGoals[i].muTrans;
      m_refState.refContact[i].muRot = this->contactGoals[i].muRot;
      m_refState.refContact[i].minFz = this->contactGoals[i].minFz;
      m_refState.refContact[i].maxFz = this->contactGoals[i].maxFz;
      m_refState.refContact[i].surface.length(this->contactGoals[i].surface.size());
      for(int j=0;j<this->contactGoals[i].surface.size();j++){
        eigen_rtm_conversions::vectorEigenToRTM(this->contactGoals[i].surface[j], m_refState.refContact[i].surface[j]);
      }
      eigen_rtm_conversions::poseEigenToRTM(this->contactGoals[i].localPose2, m_refState.refContact[i].localPose2);
    }

    if(this->feasibility == Feasibility::FEASIBLE) m_refState.feasibility = actkin_stabilizer_msgs::FEASIBLE;
    else if(this->feasibility == Feasibility::UNPREFERABLE) m_refState.feasibility = actkin_stabilizer_msgs::UNPREFERABLE;
    else if(this->feasibility == Feasibility::POSTPONED) m_refState.feasibility = actkin_stabilizer_msgs::POSTPONED;
    else m_refState.feasibility = actkin_stabilizer_msgs::INFEASIBLE;

    return;
  }
};
