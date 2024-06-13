#include "MathUtil.h"

#include <opencv2/imgproc.hpp>

namespace actkin_balancer {
  namespace mathutil {
    inline Eigen::AngleAxisd slerp(const Eigen::AngleAxisd& M1, const Eigen::AngleAxisd& M2, double r){
      // 0 <= r <= 1
      Eigen::AngleAxisd trans = Eigen::AngleAxisd(M1.inverse() * M2);
      return Eigen::AngleAxisd(M1 * Eigen::AngleAxisd(trans.angle() * r, trans.axis()));
    }

    Eigen::Vector3d calcMidPos(const std::vector<Eigen::Vector3d>& coords, const std::vector<double>& weights){
      // coordsとweightsのサイズは同じでなければならない
      double sumWeight = 0.0;
      Eigen::Vector3d midpos = Eigen::Vector3d::Zero();

      for(int i=0;i<coords.size();i++){
        if(weights[i]<=0) continue;
        midpos = ((midpos*sumWeight + coords[i]*weights[i])/(sumWeight+weights[i])).eval();
        sumWeight += weights[i];
      }
      return midpos;
    }
    Eigen::Matrix3d calcMidRot(const std::vector<Eigen::Matrix3d>& coords, const std::vector<double>& weights){
      // coordsとweightsのサイズは同じでなければならない
      double sumWeight = 0.0;
      Eigen::AngleAxisd midrot = Eigen::AngleAxisd::Identity();

      for(int i=0;i<coords.size();i++){
        if(weights[i]<=0) continue;
        midrot = mathutil::slerp(midrot, Eigen::AngleAxisd(coords[i]), weights[i]/(sumWeight+weights[i]));
        //midrot = midrot.slerp(weights[i]/(sumWeight+weights[i]),Eigen::Quaterniond(coords[i])); // quaternionのslerpは、90度回転した姿勢で不自然な遠回り補間をするので使ってはならない
        sumWeight += weights[i];
      }
      return midrot.toRotationMatrix();
    }
    Eigen::Isometry3d calcMidCoords(const std::vector<Eigen::Isometry3d>& coords, const std::vector<double>& weights){
      // coordsとweightsのサイズは同じでなければならない
      double sumWeight = 0.0;
      Eigen::Isometry3d midCoords = Eigen::Isometry3d::Identity();

      for(int i=0;i<coords.size();i++){
        if(weights[i]<=0) continue;
        midCoords.translation() = ((midCoords.translation()*sumWeight + coords[i].translation()*weights[i])/(sumWeight+weights[i])).eval();
        midCoords.linear() = mathutil::slerp(Eigen::AngleAxisd(midCoords.linear()), Eigen::AngleAxisd(coords[i].linear()),(weights[i]/(sumWeight+weights[i]))).toRotationMatrix();
        //midCoords.linear() = Eigen::Quaterniond(midCoords.linear()).slerp(weights[i]/(sumWeight+weights[i]),Eigen::Quaterniond(coords[i].linear())).toRotationMatrix(); // quaternionのslerpは、90度回転した姿勢で不自然な遠回り補間をするので使ってはならない
        sumWeight += weights[i];
      }
      return midCoords;
    }

    Eigen::Matrix3d orientCoordToAxis(const Eigen::Matrix3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis){
      // axisとlocalaxisはノルムが1, mは回転行列でなければならない.
      // axisとlocalaxisがピッタリ180反対向きの場合、回転方向が定まらないので不安定
      Eigen::AngleAxisd m_ = Eigen::AngleAxisd(m); // Eigen::Matrix3dの空間で積算していると数値誤差によってだんたん回転行列ではなくなってくるので
      Eigen::Vector3d localaxisdir = m_ * localaxis;
      Eigen::Vector3d cross = localaxisdir.cross(axis);
      double dot = std::min(1.0,std::max(-1.0,localaxisdir.dot(axis))); // acosは定義域外のときnanを返す
      if(cross.norm()==0){
        if(dot == -1) return Eigen::Matrix3d(-m);
        else return Eigen::Matrix3d(m_);
      }else{
        double angle = std::acos(dot); // 0~pi
        Eigen::Vector3d axis = cross.normalized(); // include sign
        return Eigen::Matrix3d(Eigen::AngleAxisd(angle, axis) * m_);
      }
    }
    Eigen::Isometry3d orientCoordToAxis(const Eigen::Isometry3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis){
      Eigen::Isometry3d ret = m;
      ret.linear() = mathutil::orientCoordToAxis(ret.linear(), axis, localaxis);
      return ret;
    }

    void calcConvexHull(const std::vector<Eigen::Vector2d>& contours, std::vector<Eigen::Vector2d>& hull) {
      hull.clear();
      if(contours.size()==0) return;

      std::vector<cv::Point2d> contours_;
      for(int i=0;i<contours.size();i++){
        contours_.emplace_back(contours[i][0],contours[i][1]);
      }

      std::vector<cv::Point2d> hull_;
      cv::convexHull(contours_, hull_);
      for(int i=0;i<hull_.size();i++){
        hull.emplace_back(hull_[i].x,hull_[i].y);
      }
    }
  };
};

