#pragma once
#ifndef IIWA_PKGS_GRASPING
#define IIWA_PKGS_GRASPING

#include <MoveItNode.hpp>
#include <utils.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

class GraspNode {
private:
	ros::NodeHandle node_handle;
	ros::Publisher gripper_command_publisher;

	enum State {START, PREGRASP, GRASP, FINISH, FAIL};
	State current_state = START;

	std_msgs::String gripper_command;

	ros::Rate* loop_rate_ = new ros::Rate(1);

	geometry_msgs::PoseStamped grasp_point, pre_grasp_point;
	int graspWidth;

	//#### ROBOTIQ
#ifdef REFLEX
	float gripper_offset   = 0.225;
	std::vector<double> gripper_offsets = {0, 0,-0.225};
#endif
#ifndef REFLEX
	float gripper_offset   = 0.39;
	// std::vector<double> gripper_offsets = {-0.01, -0.005,-0.38};
	std::vector<double> gripper_offsets = {-0.0, -0.0,-0.38};
#endif
	// std::vector<double> gripper_offsets = {0.00,0.00,-0.39};
	// std::vector<double> gripper_offsets = {0.00,0.00,-0.37};
	// std::vector<double> gripper_offsets = {0, 0,-0.38};
	// std::vector<double> gripper_offsets = {-0.002 -0.005,-0.39};
	float pre_grasp_height = 0.15;
	// float error_margin	   = 0.03;
	float error_margin	   = 0.00;

	bool planning_success;

	int gripper_opening;

	ros::Subscriber gripper_force_feedback_sub;
	void GripperForceFeedbackCB(const std_msgs::String::ConstPtr& msg);


public:
	GraspNode();
	void setGraspPoses(geometry_msgs::PoseStamped point);
	void setGraspWidth(double size);
	bool grasp(geometry_msgs::PoseStamped point, geometry_msgs::PoseStamped place_pose,double size,MoveItNode& moveit_node, iiwa_ros::iiwaRos& my_iiwa);
	bool grasp(geometry_msgs::PoseStamped point, double size,MoveItNode& moveit_node, iiwa_ros::iiwaRos& my_iiwa);
	void publish_command(std::string command);
	std::string get_state(State current_state);
	//Function to apply local offset frame of point
	geometry_msgs::PoseStamped offsetPoseInLocalFrame(geometry_msgs::PoseStamped point, std::vector<double> local_xyz_offsets);
	void set_grasp_opening(int opening)
	{
		gripper_opening = opening;
	}
	bool gripper_force_feedback;

	enum GRASPING_FAILURE_MODE
	{PREGRASP_MP,GRASP_MP,FINGER_CLOSURE,RETRACT_MP,SUCCESS};

	GRASPING_FAILURE_MODE grasping_failure_mode;
};



#endif