#include <iostream>
#include "se3.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

int main(){
    SE3 pose(1, 0, 0, 0, 0, 0);
    SE3 pose2(2, 0, 0, 0, 0, 0);
    SE3 pose3(3, 0, 0, 0, 0, 0);
    Eigen::Vector3d v(4, 0, 0);
    pose = pose * pose2;
    std::cout << pose.toVector() << std::endl << std::endl;
    pose2 *= pose2;
    std::cout << pose2.toVector() << std::endl << std::endl;
    v = pose3 * v;
    std::cout << v << std::endl << std::endl;
    std::cout << pose3.inverse().toVector() << std::endl << std::endl;
}
