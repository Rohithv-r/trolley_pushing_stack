#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/topic.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Pose.h>



///////////////////////////////// HELPER FUNCTIONS //////////////////////////////////////////////

static geometry_msgs::Vector3 rpyFromQuat(geometry_msgs::Quaternion quat){
    // tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Quaternion q1(quat.x, quat.y, quat.z, quat.w);

    // tf::Matrix3x3 m(q);
    tf2::Matrix3x3 m(q1);
    geometry_msgs::Vector3 v;
    m.getRPY(v.x, v.y, v.z);
    return v;
}

static geometry_msgs::Quaternion quatFromRPY(double roll, double pitch, double yaw){
    // tf::Quaternion quat;
    tf2::Quaternion quat;
    geometry_msgs::Quaternion q;
    quat.setRPY(roll, pitch, yaw);
    q.x = quat.x(); q.y = quat.y(); q.z = quat.z(); q.w = quat.w();
    return q;
}


///////////////////////////////// CLASS DEFINITION //////////////////////////////////////////////

class pr2Arm{
  public:
    explicit pr2Arm(ros::NodeHandlePtr& nh, std::string arm_side) : _nh(nh), side(arm_side){
      side_prefix = std::string(1,side[0]);
      planning_group_name = "both_arms";
      // jointPositionPublisher = _nh->advertise<trajectory_msgs::JointTrajectory>( side_prefix+std::string("_arm_controller/command"), 10);
      both_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_name);
      l_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>("left_arm");
      r_move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>("right_arm");
      kinematic_state = both_move_group_interface->getCurrentState();
      both_joint_model_group = kinematic_state->getJointModelGroup(planning_group_name);
      l_joint_model_group = l_move_group_interface->getCurrentState()->getJointModelGroup("left_arm");
      r_joint_model_group = r_move_group_interface->getCurrentState()->getJointModelGroup("right_arm");
      r_gripper_client = boost::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("/r_gripper_controller/follow_joint_trajectory");
      l_gripper_client = boost::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("/l_gripper_controller/follow_joint_trajectory");
      // std::vector<std::string> eefs = both_joint_model_group->getAttachedEndEffectorNames();
      // eef_frame = move_group_interface->getEndEffectorLink();
      // std::cout << "End effector names are : " << eefs[0] << " " << eefs[1] << std::endl;
      // std::cout << "Planning frame is : " << both_move_group_interface->getPlanningFrame() << std::endl;
      // publishToJointPositions(std::vector<double>{0.2, 0.2, 0.2, -0.2, 0.2, -0.2, 0.2});
      std::vector<double> initConfig = {0.0, 0.9, 0.0, -2.5, 0.0, -0.1, 0.0,  0.0, 0.9, 0.0, -2.5, 0.0, -0.1, 0.0};
      // moveToJointPositions(initConfig, true);
      // std::cout << "Press Enter to grasp the cart" << std::endl;
      // std::cin.get();
      std::vector<double> r_initConfig = {0.0, 0.9, 0.0, -2.5, 0.0, -0.1, 0.0};
      // std::vector<double> l_initConfig = {-0.46, 0.11, -0.83, -1.75, -2.92, -1.33, 0.82};
      std::vector<double> l_initConfig = {0.0, 0.36, 0.0, -1.75, -3.13, -1.4, -1.6};
      r_initConfig = l_initConfig;
      r_initConfig[0] = -r_initConfig[0];
      r_initConfig[2] = -r_initConfig[2];
      r_initConfig[6] = -r_initConfig[6];
      l_initConfig.insert(l_initConfig.end(), r_initConfig.begin(), r_initConfig.end());
      // if (side_prefix == "r"){
      //   initConfig[0] = -initConfig[0];
      //   initConfig[2] = -initConfig[2];
      //   initConfig[6] = -initConfig[6];
      // }
      moveToJointPositions(l_initConfig, true);
      std::cout << "Returned from moveToJointPositions" << std::endl;
      // ros::topic::waitForMessage<control_msgs::FollowJointTrajectoryActionResult>("/l_arm_controller/follow_joint_trajectory/result");
      std::cout << "Closing gripper in 1 sec" << std::endl;
      ros::Duration(1).sleep();
      closeGrippers();


      geometry_msgs::Pose targetPose;
      targetPose.position.x = 0.483;
      targetPose.position.y = 0.188;
      targetPose.position.z = 0.977;
      targetPose.orientation.x = 0.697;
      targetPose.orientation.y = 0.008;
      targetPose.orientation.z = 0.001;
      targetPose.orientation.w = 0.717;
      // processTargetPose(targetPose);
      // moveToPose(targetPose, true);
      // moveToPose(processTargetPose(targetPose), true);
      // targetPose = move_group_interface->getCurrentPose().pose;
      // std::cout << "Current eef pose = " << targetPose.position.x << " " << targetPose.position.y << " " << targetPose.position.z << " " <<
      //   targetPose.orientation.x << " " << targetPose.orientation.y << " " << targetPose.orientation.z << " " << targetPose.orientation.w << std::endl;
      std::cout << "Starting while loop" << std::endl;
      ros::Rate rate(10);
      while (_nh->ok()){
        ros::spinOnce();
        rate.sleep();
      }
      std::cout << "Ending while loop" << std::endl;
    }


  private:
  // Variables
    ros::NodeHandlePtr _nh;
    std::string side, side_prefix, planning_group_name, eef_frame;
    moveit::planning_interface::MoveGroupInterfacePtr l_move_group_interface, r_move_group_interface, both_move_group_interface;
    moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface;
    const robot_state::JointModelGroup *both_joint_model_group, *l_joint_model_group, *r_joint_model_group;
    moveit::core::RobotStatePtr kinematic_state;
    ros::Publisher jointPositionPublisher;
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> r_gripper_client, l_gripper_client;

  // Member functions
    geometry_msgs::Pose processTargetPose(geometry_msgs::Pose desPose);
    bool compareCurrentToTargetJointPositions(std::vector<double> target, double tolerance = 0.1);
    bool publishToJointPositions(std::vector<double> joint_positions, double time_from_start = 2);  // This is a blocking function!
    bool moveToJointPositions(std::vector<double> j_values, bool blocking=false);
    bool moveToPose(geometry_msgs::Pose target_pose, bool blocking = false, bool gripperMonitoring = false);
    bool closeGrippers();


};





geometry_msgs::Pose pr2Arm::processTargetPose(geometry_msgs::Pose desPose){
    // geometry_msgs::PoseStamped p, world_pose;
    // p.header.frame_id = eef_frame;
    // // desPose.position.x = 0.18;
    // p.pose.position.x = 0.18;
    tf2::Quaternion q1(desPose.orientation.x, desPose.orientation.y, desPose.orientation.z, desPose.orientation.w);
    tf2::Matrix3x3 m(q1);
    double x_shift = -0.18;
    desPose.position.x += m[0][0]*x_shift;
    desPose.position.y += m[1][0]*x_shift;
    desPose.position.z += m[2][0]*x_shift;
    std::cout << "Processed x, y, z = " << desPose.position.x << " " << desPose.position.y << " " << desPose.position.z << std::endl;
    return desPose;
}



bool pr2Arm::compareCurrentToTargetJointPositions(std::vector<double> target, double tolerance){
    std::vector<double> cur = both_move_group_interface->getCurrentJointValues();
    for (int i = 0; i < target.size(); i++){
      if ( abs(cur[i] - target[i]) < tolerance )
        continue;
      else
        return false;
    }
    return true;
}


bool pr2Arm::publishToJointPositions(std::vector<double> joint_positions, double time_from_start){
    trajectory_msgs::JointTrajectory msg;
    std::cout << "Publishing to joint positions!" << std::endl;
    msg.joint_names.clear();
    msg.joint_names.push_back(side_prefix + "_shoulder_pan_joint");
    msg.joint_names.push_back(side_prefix + "_shoulder_lift_joint");
    msg.joint_names.push_back(side_prefix + "_upper_arm_roll_joint");
    msg.joint_names.push_back(side_prefix + "_elbow_flex_joint");
    msg.joint_names.push_back(side_prefix + "_forearm_roll_joint");
    msg.joint_names.push_back(side_prefix + "_wrist_flex_joint");
    msg.joint_names.push_back(side_prefix + "_wrist_roll_joint");
    msg.points.resize(1);
    
    msg.points[0].positions = joint_positions;
    msg.points[0].time_from_start = ros::Duration(time_from_start);
    // ros::Rate rate(10);
    bool too_much_time = false;
    ros::Time start_time = ros::Time::now();
    int k = 0;
    while (!compareCurrentToTargetJointPositions(joint_positions, 0.05) && !(too_much_time = (ros::Duration(10) < (ros::Time::now() - start_time))) ){
      jointPositionPublisher.publish(msg);
      ros::spinOnce();
      // rate.sleep();
    }
    if(!too_much_time){
      std::cout << "PublishToJointPositions returning true!" << std::endl;
      return true;
    }
    // ROS_ERROR_STREAM("Took more than 10 secs to publish to joint positions : " << (ros::Time::now() - start_time).toSec());
    ROS_ERROR_STREAM("Took more than 10 secs to publish to joint positions");
    return false;
}


bool pr2Arm::moveToJointPositions(std::vector<double> j_values, bool blocking){
    both_move_group_interface->setJointValueTarget(j_values);
    bool success;
    std::cout << "moveToJointPositions called!" << std::endl;
    if (!blocking){
      success = (both_move_group_interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      std::cout << "Asyncmove() called. Success = " << success << std::endl;
    }
    else{
        success = (both_move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    return success;
}


bool pr2Arm::moveToPose(geometry_msgs::Pose target_pose, bool blocking, bool gripperMonitoring){
    std::cout << "Movetopose called!" << std::endl;
    bool success;
    moveit::core::RobotStatePtr l_kin_state, r_kin_state;
    l_kin_state = l_move_group_interface->getCurrentState();
    r_kin_state = r_move_group_interface->getCurrentState();
    bool l_found_ik = l_kin_state->setFromIK(l_joint_model_group, target_pose);
    bool r_found_ik = r_kin_state->setFromIK(r_joint_model_group, r_move_group_interface->getCurrentPose().pose);
    // if (blocking){
    //   if (gripperMonitoring)
    //     success = moveToPoseWhileCM(target_pose);
    //   else
    //     success = moveToPoseWhileCMandGM(target_pose);
    // }
    // else
    // {  
      // move_group_interface->setEndEffector("right_end_effector");
      // move_group_interface->setEndEffector("left_end_effector");
    std::vector<double> l_joint_vals, r_joint_vals;
    if (l_found_ik && r_found_ik){
      l_kin_state->copyJointGroupPositions(l_joint_model_group, l_joint_vals);
      r_kin_state->copyJointGroupPositions(r_joint_model_group, r_joint_vals);
    }
    else{
      std::cout << "IK failed!" << l_found_ik << " " << r_found_ik << std::endl;
      return false;
    }
    l_joint_vals.insert(l_joint_vals.end(), r_joint_vals.begin(), r_joint_vals.end());
    return moveToJointPositions(l_joint_vals, blocking);
      
      // move_group_interface->setEndEffectorLink("l_wrist_roll_link");
      // std::cout << "End effector link : " << move_group_interface->getEndEffectorLink() << std::endl;
      // move_group_interface->setPoseTarget(target_pose, "l_wrist_roll_link");
      // moveit::planning_interface::MoveGroupInterface::Plan plan;
      // int c = 0;
      // success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      // while (!success && c<5){
      //   success = (move_group_interface->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      //   // if (success and blocking)
      //   //   success = (move_group_interface->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      //   if (success and !blocking)
      //     success = (move_group_interface->asyncExecute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      //   if (blocking)
      //     success = (move_group_interface->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      //   c++;
      // }
      // if (c >= 5){
      //   std::cout << "Could not find plan for pose : " << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << std::endl;
      // }
    // }
    return success;
}


bool pr2Arm::closeGrippers(){
    std::cout << "closeGrippers called!" << std::endl;
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("l_left_gripper_finger_joint");
    goal.trajectory.joint_names.push_back("l_right_gripper_finger_joint");
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions.resize(2);
    goal.trajectory.points[0].positions[0] = 0.05;
    goal.trajectory.points[0].positions[1] = 0.05;
    // goal.trajectory.points[0].velocities.resize(2, 1.0);
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);
    l_gripper_client->sendGoal(goal);
    goal.trajectory.joint_names.clear();
    goal.trajectory.joint_names.push_back("r_left_gripper_finger_joint");
    goal.trajectory.joint_names.push_back("r_right_gripper_finger_joint");
    r_gripper_client->sendGoal(goal);

}