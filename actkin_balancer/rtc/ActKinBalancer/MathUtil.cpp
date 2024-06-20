#include "MathUtil.h"

#include <opencv2/imgproc.hpp>
#include <limits>


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
      if(contours.size()==0) {
        hull.clear();
        return;
      }

      std::vector<cv::Point2f> contours_;
      for(int i=0;i<contours.size();i++){
        contours_.emplace_back(contours[i][0],contours[i][1]);
      }

      std::vector<cv::Point2f> hull_;
      cv::convexHull(contours_, hull_);

      hull.clear();
      for(int i=0;i<hull_.size();i++){
        hull.emplace_back(hull_[i].x,hull_[i].y);
      }
    }

    std::vector<Eigen::Vector2d> calcIntersectConvexHull(const std::vector<Eigen::Vector2d>& P, const std::vector<Eigen::Vector2d>& Q) {
      if(P.size() == 0 || Q.size() == 0) return std::vector<Eigen::Vector2d>();

      std::vector<cv::Point2f> P_;
      for(int i=0;i<P.size();i++) P_.emplace_back(P[i][0],P[i][1]);
      std::vector<cv::Point2f> Q_;
      for(int i=0;i<Q.size();i++) Q_.emplace_back(Q[i][0],Q[i][1]);

      if(P_.size() == 1){
        if(cv::pointPolygonTest(Q_,P_[0],false) >= 0.0) return P;
        else return std::vector<Eigen::Vector2d>();
      }
      if(Q_.size() == 1){
        if(cv::pointPolygonTest(P_,Q_[0],false) >= 0.0) return Q;
        else return std::vector<Eigen::Vector2d>();
      }

      std::vector<cv::Point2f> ret_;
      // 単精度浮動小数点にしないとエラーが出る
      cv::intersectConvexConvex(P_,Q_,ret_,true);

      std::vector<Eigen::Vector2d> ret;
      for(int i=0;i<ret.size();i++) ret.emplace_back(ret_[i].x,ret_[i].y);
      return ret;
    }

    bool isInsideHull(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& contours) {
      if(contours.size()==0) {
        return false;
      }

      std::vector<cv::Point2f> contours_;
      for(int i=0;i<contours.size();i++){
        contours_.emplace_back(contours[i][0],contours[i][1]);
      }

      cv::Point2f pt(p[0],p[1]);
      double result = cv::pointPolygonTest(contours_,pt,false);
      return result >= 0.0;
    }

    Eigen::Vector2d calcNearestPointOfHull(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& contours){
      if(contours.size() == 0)  return p;
      if(contours.size() == 1) return contours[0];

      std::vector<cv::Point2f> contours_;
      for(int i=0;i<contours.size();i++){
        contours_.emplace_back(contours[i][0],contours[i][1]);
      }

      cv::Point2f p_(p[0],p[1]);
      double result = cv::pointPolygonTest(contours_,p_,false);

      if(result >= 0.0) return p; // p is inside contours

      double minDistance = std::numeric_limits<double>::max();
      Eigen::Vector2d nearestPoint;
      for (int i = 0; i < contours.size(); i++) {
        const Eigen::Vector2d& p1 = contours[i], p2 = contours[(i+1)%contours.size()];
        double dot = (p2 - p1).dot(p - p1);
        if(dot <= 0) { // p1が近い
          double distance = (p - p1).norm();
          if(distance < minDistance){
            minDistance = distance;
            nearestPoint = p1;
          }
        }else if(dot >= (p2 - p1).squaredNorm()) { // p2が近い
          double distance = (p - p2).norm();
          if(distance < minDistance){
            minDistance = distance;
            nearestPoint = p2;
          }
        }else { // 直線p1 p2におろした垂線の足が近い
          Eigen::Vector2d p1Top2 = (p2 - p1).normalized();
          Eigen::Vector2d p3 = p1 + (p - p1).dot(p1Top2) * p1Top2;
          double distance = (p - p3).norm();
          if(distance < minDistance){
            minDistance = distance;
            nearestPoint = p3;
          }
        }
      }
      return nearestPoint;
    }

    double findExtremes(const std::vector<Eigen::Vector2d> vertices, const Eigen::Vector2d& dir, std::vector<Eigen::Vector2d>& ret) {
      ret.clear();
      double maxValue = - std::numeric_limits<double>::max();
      for(int i=0;i<vertices.size();i++){
        double value = vertices[i].dot(dir);
        if(value > maxValue + 1e-4){
          ret.clear();
          ret.push_back(vertices[i]);
          maxValue = value;
        }else if(value >= maxValue - 1e-4 && value <= maxValue + 1e-4){
          ret.push_back(vertices[i]);
        }
      }
      return maxValue;
    }
  };
};

