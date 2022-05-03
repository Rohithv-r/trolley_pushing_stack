#include <pr2_grasping_pkg/pr2_arm_server.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "rightArm_node");
    ros::NodeHandlePtr node(new ros::NodeHandle);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    pr2Arm rightArm(node, "right");
    std::cout << "Quitting right_arm_node!" << std::endl;
    return 0;
}