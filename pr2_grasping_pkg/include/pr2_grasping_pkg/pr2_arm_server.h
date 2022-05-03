#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

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
      planning_group_name = side + "_arm";
      jointPositionPublisher = _nh->advertise<trajectory_msgs::JointTrajectory>( side_prefix+std::string("_arm_controller/command"), 10);
      move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group_name);
      joint_model_group = move_group_interface->getCurrentState()->getJointModelGroup(planning_group_name);
      eef_frame = move_group_interface->getEndEffectorLink();
      std::cout << "End effector link is : " << eef_frame << std::endl;
      std::cout << "Planning frame is : " << move_group_interface->getPlanningFrame() << std::endl;
      // publishToJointPositions(std::vector<double>{0.2, 0.2, 0.2, -0.2, 0.2, -0.2, 0.2});
      std::vector<double> initConfig = {0.2, 0.2, 0.2, -0.2, 0.2, -0.2, 0.2};
      if (side_prefix == "r"){
        initConfig[0] = -initConfig[0];
        initConfig[2] = -initConfig[2];
        initConfig[6] = -initConfig[6];
      }
      // head tilt : 0.26
      // moveToJointPositions(initConfig, true);
      geometry_msgs::Pose targetPose;
      targetPose.position.x = 0.630;
      if (side_prefix == "r"){
        targetPose.position.x = -0.63;
      }
      targetPose.position.y = 0.165;
      targetPose.position.z = 1.199;
      targetPose.orientation.x = 0;
      targetPose.orientation.y = -0.125;
      targetPose.orientation.z = 0;
      targetPose.orientation.w = 0.992;
      // processTargetPose(targetPose);
      if (side_prefix == "l")
        moveToPose(processTargetPose(targetPose), true);
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
    moveit::planning_interface::MoveGroupInterfacePtr move_group_interface;
    moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface;
    const robot_state::JointModelGroup* joint_model_group;
    moveit::core::RobotStatePtr current_state;
    ros::Publisher jointPositionPublisher;

  // Member functions
    geometry_msgs::Pose processTargetPose(geometry_msgs::Pose desPose);
    bool compareCurrentToTargetJointPositions(std::vector<double> target, double tolerance = 0.1);
    bool publishToJointPositions(std::vector<double> joint_positions, double time_from_start = 2);  // This is a blocking function!
    bool moveToJointPositions(std::vector<double> j_values, bool blocking=false);
    bool moveToPose(geometry_msgs::Pose target_pose, bool blocking = false, bool gripperMonitoring = false);


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
    std::vector<double> cur = move_group_interface->getCurrentJointValues();
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
    move_group_interface->setJointValueTarget(j_values);
    bool success;
    if (!blocking){
      success = (move_group_interface->asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      std::cout << "Asyncmove() called. Success = " << success << std::endl;
    }
    else{
        success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    return success;
}


bool pr2Arm::moveToPose(geometry_msgs::Pose target_pose, bool blocking, bool gripperMonitoring){
    std::cout << "Movetopose called!" << std::endl;
    bool success;
    // if (blocking){
    //   if (gripperMonitoring)
    //     success = moveToPoseWhileCM(target_pose);
    //   else
    //     success = moveToPoseWhileCMandGM(target_pose);
    // }
    // else
    // {  
      move_group_interface->setPoseTarget(target_pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      int c = 0;
      success = (move_group_interface->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
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