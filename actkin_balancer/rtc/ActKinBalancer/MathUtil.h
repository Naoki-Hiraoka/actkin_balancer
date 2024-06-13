#ifndef ACTKINBALANCER_MATHUTIL_H
#define ACTKINBALANCER_MATHUTIL_H

#include <Eigen/Eigen>
#include <vector>

namespace actkin_balancer {
  namespace mathutil {
    // coordsとweightsのサイズは同じでなければならない
    Eigen::Vector3d calcMidPos(const std::vector<Eigen::Vector3d>& coords, const std::vector<double>& weights);

    // coordsとweightsのサイズは同じでなければならない
    Eigen::Matrix3d calcMidRot(const std::vector<Eigen::Matrix3d>& coords, const std::vector<double>& weights);

    // coordsとweightsのサイズは同じでなければならない
    Eigen::Isometry3d calcMidCoords(const std::vector<Eigen::Isometry3d>& coords, const std::vector<double>& weights);

    // axisとlocalaxisはノルムが1, mは回転行列でなければならない.
    // axisとlocalaxisがピッタリ180反対向きの場合、回転方向が定まらないので不安定
    Eigen::Matrix3d orientCoordToAxis(const Eigen::Matrix3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis = Eigen::Vector3d::UnitZ());
    Eigen::Isometry3d orientCoordToAxis(const Eigen::Isometry3d& m, const Eigen::Vector3d& axis, const Eigen::Vector3d& localaxis = Eigen::Vector3d::UnitZ());


    void calcConvexHull(const std::vector<Eigen::Vector2d>& contours, std::vector<Eigen::Vector2d>& hull);
  };
};

#endif
