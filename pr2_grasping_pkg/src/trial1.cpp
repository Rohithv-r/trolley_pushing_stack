#include <pr2_grasping_pkg/pr2_both_arms_server.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "trial_node");
    ros::NodeHandlePtr node(new ros::NodeHandle);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    pr2Arm leftArm(node, "asdasd");
    std::cout << "Quitting trial_node!" << std::endl;
    return 0;
}