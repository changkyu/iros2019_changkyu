#include "ros/ros.h"
//#include "test_pkg/GraspPlanning.h"
#include "grasping/GraspPlanning.h"
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "grasp_planning_client");
	ros::NodeHandle nh;

	//YoubotGrasp youbot_grasp(nh);
	// std::vector<fcl::Vector3<double>> vertices_cube_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Vector3<double>> vertices_collision_gripper_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Vector3<double>> vertices_base_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Vector3<double>> vertices_floor_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Triangle> triangles_base_ = std::vector<fcl::Triangle>();
 //  std::vector<fcl::Triangle> triangles_floor_ = std::vector<fcl::Triangle>();
 //  std::vector<fcl::Triangle> triangles_cube_ = std::vector<fcl::Triangle>();
 //  std::vector<fcl::Triangle> triangles_collision_gripper_ = std::vector<fcl::Triangle>();
//	ros::ServiceClient grasp_planning_client = nh.serviceClient<test_pkg::GraspPlanning>("CollisionGraspPlanning");
	ros::ServiceClient grasp_planning_client = nh.serviceClient<grasping::GraspPlanning>("CollisionGraspPlanning");
	grasping::GraspPlanning srv;
	geometry_msgs::PoseStamped target;
	target.pose.position.x = 0.40695;
	target.pose.position.y = 0.324461;
	target.pose.position.z = -0.0553423;
	target.pose.orientation.x = 1;
	target.pose.orientation.y =0;
	target.pose.orientation.z = 0;
	target.pose.orientation.w = 0;

	srv.request.binId = 2;
	srv.request.target_pose = target.pose;

	if (grasp_planning_client.call(srv))
  {
    //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

	//ROS_INFO("Ready to plan grasp.");
	ros::spin();

	return 0;
}
