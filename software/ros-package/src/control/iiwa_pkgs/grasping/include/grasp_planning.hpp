#ifndef __KUKA_GRASP_H__
#define __KUKA_GRASP_H__

#include "opencv2/opencv.hpp"

#include "grasping/GraspPlanning.h"  // service file
#include "grasping/GraspSuccess.h"  // service file

#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <vector>
#include <tf/tf.h>
//#include <Quaternion.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>
class CollisionRegion
{
public:
	double x_min_;
	double y_min_;
	double z_min_;
	double x_max_;
	double y_max_;
	double z_max_;

	CollisionRegion();
	CollisionRegion(double x_min, double x_max,double y_min,double y_max,double z_min,double z_max){
		x_min_ = x_min;
		x_max_ = x_max;
		y_min_ = y_min;
		y_max_ = y_max;
		z_min_ = z_min;
		z_max_ = z_max;
	}
};


class KukaGrasp
{
private:

	ros::NodeHandle* node_;
	//! A pointer to the grasp planner
	


	//object file name
	//std::string object_file_name_;

	// //object pose
	// Eigen::Vector3d object_pose;

	// //object position
	// Eigen::Vector3d object_position;

	//environment representation
	//moveit_msgs::PlanningScene planning_scene_env_;

	//robot representatoin
	//robot_state::RobotState* robot_state_ptr_;

	
public:
/*
 * constructor, destructor 
 */
    KukaGrasp(ros::NodeHandle &node);
    //KukaGrasp(GraspPlanner *grasp_planner_in, FeasibilityChecker *feasibility_checker_in, GraspEvaluator *grasp_evaluator_in);
    ~KukaGrasp();

    /*call back function for the whole grasp planning pipe line*/
    bool graspPlanningServiceCB(grasping::GraspPlanning::Request &req, grasping::GraspPlanning::Response &res);
    bool graspSuccessServiceCB(grasping::GraspSuccess::Request &req, grasping::GraspSuccess::Response &res);
    
    void convert3dUnOrganized(cv::Mat &objDepth, Eigen::Matrix3f &camIntrinsic, pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud);

	void toTransformationMatrix(Eigen::Matrix4f& camPose, std::vector<double> camPose7D);
	void readDepthImage(cv::Mat &depthImg, std::string path);
void InverseTransformationMatrix(Eigen::Matrix4f &origin, Eigen::Matrix4f &inverse);

    // /*grasp planning*/
    // bool planGrasp();

    // /*feasibility checking*/
    // bool checkFeasibility();

    // /*grasp evaluation*/
    // void evaluateGrasp();

    // /*grasp clustering*/
    // void clusterGrasp();

    // convert data from request message
    // void convertFromMessage();

    // /*convert fata to response message*/
    // void convertToMessage();
    
    // bool compare_quality(const plannedGrasp* first, const plannedGrasp* second);

};










#endif