#include "MathUtil.h"
#include <iostream>

int main(void){
  std::vector<Eigen::Vector2d> vs;
  vs.emplace_back(1.0,1.0);
  vs.emplace_back(-1.0,-1.0);
  vs.emplace_back(-1.0,1.0);
  vs.emplace_back(1.0,-1.0);
  std::vector<Eigen::Vector2d> hull;
  actkin_balancer::mathutil::calcConvexHull(vs, hull);
  Eigen::Vector2d p(1.01,0.0);
  bool inside = actkin_balancer::mathutil::isInsideHull(p, hull);

  std::cerr << "vs" << std::endl;
  for(int i=0;i<vs.size();i++){
    std::cerr << vs[i].transpose() << std::endl;
  }
  std::cerr << "hull" << std::endl;
  for(int i=0;i<hull.size();i++){
    std::cerr << hull[i].transpose() << std::endl;
  }
  std::cerr << "inside " << inside << std::endl;
  return 0;
}
