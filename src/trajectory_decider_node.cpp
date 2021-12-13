#include "trajectory_decider/trajectory_decider.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_decider");
    TrajectoryDecider trajectory_decider;
    trajectory_decider.process();
    return 0;
}
