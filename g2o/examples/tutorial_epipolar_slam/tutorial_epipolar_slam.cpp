#include <iostream>
#include "se3.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

int main(){
    SE3 pose(1, 0, 0, 0, 0, 0);
    SE3 pose2(2, 0, 0, 0, 0, 0);
    pose = pose * pose2;
    std::cout << pose.toVector() << std::endl;
}
