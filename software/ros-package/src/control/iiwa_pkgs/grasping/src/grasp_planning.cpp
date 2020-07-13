#include <grasp_planning.hpp>
#include<bits/stdc++.h>
//  #define _DEBUG
ros::Publisher marker_pub;
ros::Publisher cube_pub;
ros::Publisher cloud_pub;
float _x,_y,_z;
KukaGrasp::KukaGrasp(ros::NodeHandle &node)
{
  node_ = &node;
  marker_pub = node_->advertise<visualization_msgs::Marker>("Collision_checking_region", 1);
  cube_pub = node_->advertise<visualization_msgs::Marker>("success_region", 1);
  cloud_pub = node_->advertise<pcl::PointCloud<pcl::PointXYZ>> ("graspin_point_cloud", 1);
#ifdef _DEBUG
  std::cout << "DBG: KukaGrasp built." << std::endl;
#endif
}

// 

KukaGrasp::~KukaGrasp()
{
#ifdef _DEBUG
  std::cout << "DBG: KukaGrasp destroyed." << std::endl;
#endif
}

void publish_cylinder(geometry_msgs::Pose pose)
{

  // char markerId[30];
// sprintf(markerId, "Object%04d", rand()%9999);
  std::cout<<"Publishing marker..."<<std::endl;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/world";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  // geometry_msgs::Point p1, p2;
  // float arrow_size = 0.1;
  // p1.x = global_surface_center_world[0];
  // p1.y = global_surface_center_world[1];
  // p1.z = global_surface_center_world[2];
  // p2.x = global_surface_center_world[0] + arrow_size*global_secondary_axis[0];
  // p2.y = global_surface_center_world[1] + arrow_size*global_secondary_axis[1];
  // p2.z = global_surface_center_world[2] + arrow_size*global_secondary_axis[2];
  // marker.points.push_back(p1);
  // marker.points.push_back(p2);

  marker.pose = pose;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.7;

  // std::cout << p1.x << " " << p1.y << " " << p1.z << " " << p2.x << " " << p2.y << " " << p2.z << std::endl;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.15;

  marker.lifetime = ros::Duration(10000);
  marker_pub.publish(marker);
  ros::spinOnce();


  visualization_msgs::Marker cube;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  cube.header.frame_id = "/world";
  cube.header.stamp = ros::Time::now();

  // Set the namespace and id for this cube.  This serves to create a unique ID
  // Any cube sent with the same namespace and id will overwrite the old one
  cube.ns = "basic_cube";
  cube.id = 0;

  // Set the cube type
  cube.type = visualization_msgs::Marker::CUBE;
  cube.action = visualization_msgs::Marker::ADD;

  // geometry_msgs::Point p1, p2;
  // float arrow_size = 0.1;
  // p1.x = global_surface_center_world[0];
  // p1.y = global_surface_center_world[1];
  // p1.z = global_surface_center_world[2];
  // p2.x = global_surface_center_world[0] + arrow_size*global_secondary_axis[0];
  // p2.y = global_surface_center_world[1] + arrow_size*global_secondary_axis[1];
  // p2.z = global_surface_center_world[2] + arrow_size*global_secondary_axis[2];
  // cube.points.push_back(p1);
  // cube.points.push_back(p2);

  cube.pose = pose;
  cube.color.r = 0.0f;
  cube.color.g = 1.0f;
  cube.color.b = 0.0f;
  cube.color.a = 0.7;

  // std::cout << p1.x << " " << p1.y << " " << p1.z << " " << p2.x << " " << p2.y << " " << p2.z << std::endl;
  // Set the scale of the cube -- 1x1x1 here means 1m on a side
  cube.scale.x = 0.05;
  cube.scale.y = 0.05;
  cube.scale.z = 0.05;

  cube.lifetime = ros::Duration(10000);
  cube_pub.publish(cube);

  
  ros::spinOnce();
}


void KukaGrasp::convert3dUnOrganized(cv::Mat &objDepth, Eigen::Matrix3f &camIntrinsic, pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud){
	int imgWidth = objDepth.cols;
	int imgHeight = objDepth.rows;
	for(int u=0; u<imgHeight; u++)
		for(int v=0; v<imgWidth; v++){
			float depth = objDepth.at<float>(u,v);
			if(depth > 0.1 && depth < 2.0){
				pcl::PointXYZ pt;
				pt.x = (float)((v - camIntrinsic(0,2)) * depth / camIntrinsic(0,0));
				pt.y = (float)((u - camIntrinsic(1,2)) * depth / camIntrinsic(1,1));
				pt.z = depth;
				objCloud->points.push_back(pt);
			}
		}
}

void KukaGrasp::readDepthImage(cv::Mat &depthImg, std::string path){
	float depth_scale = 1.25; // constant scale factor for realsense sr300
	cv::Mat depthImgRaw = cv::imread(path, CV_16UC1);
	depthImg = cv::Mat::zeros(depthImgRaw.rows, depthImgRaw.cols, CV_32FC1);

	for(int u=0; u<depthImgRaw.rows; u++)
		for(int v=0; v<depthImgRaw.cols; v++){
			unsigned short depthShort = depthImgRaw.at<unsigned short>(u,v);

			float depth = (float)depthShort*depth_scale/10000;
			depthImg.at<float>(u, v) = depth;
		}
}

void KukaGrasp::toTransformationMatrix(Eigen::Matrix4f& camPose, std::vector<double> camPose7D){
	camPose(0,3) = camPose7D[0];
	camPose(1,3) = camPose7D[1];
	camPose(2,3) = camPose7D[2];
	camPose(3,3) = 1;

	Eigen::Quaternionf q;
	q.w() = camPose7D[3];
	q.x() = camPose7D[4];
	q.y() = camPose7D[5];
	q.z() = camPose7D[6];
	Eigen::Matrix3f rotMat;
	rotMat = q.toRotationMatrix();

	for(int ii = 0;ii < 3; ii++)
		for(int jj=0; jj < 3; jj++){
			camPose(ii,jj) = rotMat(ii,jj);
		}
}

void KukaGrasp::InverseTransformationMatrix(Eigen::Matrix4f &origin, Eigen::Matrix4f &inverse){
	// inverse(0,3) = - origin(0,3);
	// inverse(1,3) =  origin(1,3);
	// inverse(2,3) =  origin(2,3);
	// inverse(3,3) = 1;
	Eigen::Matrix3f rotMat = origin.block(0, 0, 3, 3);
	Eigen::Matrix3f inverse_rotMat = rotMat.inverse();
	for(int ii = 0;ii < 3; ii++)
		for(int jj=0; jj < 3; jj++){
			inverse(ii,jj) = inverse_rotMat(ii,jj);
		}
	Eigen::Vector3f temp(-origin(0, 3), -origin(1,3), -origin(2,3));
	Eigen::Vector3f final = inverse_rotMat*temp;
	inverse(0,3) = final(0);
	inverse(1,3) = final(1);
	inverse(2,3) = final(2);

	std::cout<<final(0)<<","<<final(1)<<","<<final(2)<<std::endl;

}

/*call back function for the whole grasp planning pipe line*/
bool KukaGrasp::graspPlanningServiceCB(grasping::GraspPlanning::Request &req, grasping::GraspPlanning::Response &res)
{	ROS_WARN("service call received");
	double detected_length = 0.15;
	ros::NodeHandle nh_1;
	char cam_intrinsic_topic[100];
	char cam_pose_topic[100];
	sensor_msgs::Image::ConstPtr msg_depth;
	std::string resources_path = ros::package::getPath("chimp_resources");
	system(("rosparam load " +resources_path + "/config/bin_config.yaml").c_str());
	char depth_image_topic_param[100];
	int binId = req.binId;
	XmlRpc::XmlRpcValue camIntr;
	std::vector<double> camPose7D;
	std::string depth_image_topic;
	sprintf(cam_pose_topic,"/bins/bin_%d/camera_pose", binId);
	node_->getParam(cam_pose_topic, camPose7D);
	Eigen::Matrix4f camPose = Eigen::Matrix4f::Zero(4,4);
	toTransformationMatrix(camPose, camPose7D);
	std::cout << "Camera Pose: " << std::endl << camPose << std::endl;

	// Reading camera intrinsic matrix
	sprintf(cam_intrinsic_topic,"/bins/bin_%d/camera_intrinsic", binId);
	Eigen::Matrix3f camIntrinsic = Eigen::Matrix3f::Zero(3,3);
	node_->getParam(cam_intrinsic_topic, camIntr);

	for(int32_t ii = 0; ii < camIntr.size(); ii++)
		for(int32_t jj = 0; jj < camIntr[ii].size(); jj++)
			camIntrinsic(ii, jj) = static_cast<double>(camIntr[ii][jj]);
	std::cout << "Camera Intrinsics: " << std::endl << camIntrinsic << std::endl;

	sprintf(depth_image_topic_param,"/bins/bin_%d/depth_image_topic", binId);
	node_->getParam(depth_image_topic_param, depth_image_topic);

	std::cout << "Waiting for depth image on topic: " << depth_image_topic << std::endl;
	//msg_depth = ros::topic::waitForMessage<sensor_msgs::Image>(depth_image_topic, *node_);
msg_depth = ros::topic::waitForMessage<sensor_msgs::Image>(depth_image_topic, nh_1);
	cv_bridge::CvImagePtr cv_ptr_depth;
	cv::Mat depth_image;
	try {
	   cv_ptr_depth = cv_bridge::toCvCopy(*msg_depth, (*msg_depth).encoding);
	   //depth_image = cv_ptr_depth->image;
	    cv::imwrite("/home/cm1074/Desktop/frame-000000.depth.png", cv_ptr_depth->image);
		readDepthImage(depth_image, "/home/cm1074/Desktop/frame-000000.depth.png");
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("cv_bridge exception: %s", e.what());\
	    exit(-1);
	}

	std::cout << "got depth image" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objCloudWorld(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr objCloudLeftGripper(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objCloudRightGripper(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objCloudGripper(new pcl::PointCloud<pcl::PointXYZ>);

	convert3dUnOrganized(depth_image, camIntrinsic, objCloud);
	pcl::transformPointCloud(*objCloud, *objCloudWorld, camPose);
	objCloudWorld->header.frame_id = "world";
	//cloud_pub.publish(objCloudWorld);
	ros::spinOnce();
	// for(int ii=0;ii<objCloudWorld->points.size();ii++) {
	// std::cout << objCloudWorld->points[ii].x << " " << objCloudWorld->points[ii].y << objCloudWorld->points[ii].z;
	// }

	std::vector<geometry_msgs::PoseStamped> grasp_list;
	std::vector<int> rank;
	geometry_msgs::Pose grasp_pose = req.target_pose;
	tf::Quaternion gripper_q(grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);
	tf::Matrix3x3 gripper_m(gripper_q);
	tf::Vector3 gripper_z_axis = gripper_m.getColumn(2);
	geometry_msgs::Pose published_gripper_pose = grasp_pose;
	published_gripper_pose.position.x  -= gripper_z_axis.getX()*0.5*detected_length;
	published_gripper_pose.position.y  -= gripper_z_axis.getY()*0.5*detected_length;
	published_gripper_pose.position.z  -= gripper_z_axis.getZ()*0.5*detected_length;

	publish_cylinder(published_gripper_pose);
	int collision_points_num = 0;

	double gripper_x = grasp_pose.position.x;
	double gripper_y = grasp_pose.position.y;
	double gripper_z = grasp_pose.position.z;
	Eigen::Matrix4f gripper_matrix = Eigen::Matrix4f::Zero(4,4);
	Eigen::Matrix4f world_to_gripper_pose = Eigen::Matrix4f::Zero(4,4);
	std::vector<double> gripper_pose7d = {gripper_x, gripper_y, gripper_z, grasp_pose.orientation.w, grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z};
	toTransformationMatrix(gripper_matrix, gripper_pose7d);
	std::cout << "gripper matrix: " << std::endl << gripper_matrix << std::endl;
	InverseTransformationMatrix(gripper_matrix, world_to_gripper_pose);
	objCloudGripper->header.frame_id = "world";
	std::cout << "World to gripper pose: " << std::endl << world_to_gripper_pose << std::endl;
	pcl::transformPointCloud(*objCloudWorld, *objCloudGripper, world_to_gripper_pose);
	//cloud_pub.publish(objCloudGripper);
	
	double point_x, point_y, point_z;
	std::cout<<"total num:"<<objCloudGripper->points.size()<<std::endl;
	for(int ii=0;ii<objCloudGripper->points.size();ii++) {
		//std::cout <<"||"<< objCloudGripper->points[ii].x << " " << objCloudGripper->points[ii].y << " " << objCloudGripper->points[ii].z;	
		point_x = objCloudGripper->points[ii].x;
		point_y = objCloudGripper->points[ii].y;
		point_z = objCloudGripper->points[ii].z;
		if(point_x*point_x + point_y*point_y < 0.01*0.01 && point_z<0 && point_z >-0.15){
			collision_points_num++;
		}
	}


	// int left_num = 0;
	// int right_num = 0;
	// for(int i = 0; i < 12;i++){
	// 	tf::Matrix3x3 temp_rot;
	// 	geometry_msgs::PoseStamped temp_grasp_pose;
	// 	temp_rot.setEulerYPR(360/12*i/180*3.14159, 0, 0);
	// 	tf::Matrix3x3 temp_gripper_m = gripper_m * temp_rot;
	// 	tf::Vector3 gripper_y_axis = temp_gripper_m.getColumn(1);
	// 	tf::Quaternion temp_gripper_q;
	// 	temp_gripper_m.getRotation(temp_gripper_q);
	// 	double gripper_x = grasp_pose.position.x;
	// 	double gripper_y = grasp_pose.position.y;
	// 	double gripper_z = grasp_pose.position.z;
	// 	temp_grasp_pose.position.x = gripper_x;
	// 	temp_grasp_pose.position.y = gripper_y;
	// 	temp_grasp_pose.position.z = gripper_z;
	// 	temp_grasp_pose.orientation.x = temp_gripper_q.x();
	// 	temp_grasp_pose.orientation.y = temp_gripper_q.y();
	// 	temp_grasp_pose.orientation.z = temp_gripper_q.z();
	// 	temp_grasp_pose.orientation.w = temp_gripper_q.getW();

	// 	tf::Vector3 left_center(gripper_x + 0.08*gripper_y_axis.getX(), gripper_y + 0.08*gripper_y_axis.getY(), gripper_z + 0.08*gripper_y_axis.getZ());
	// 	tf::Vector3 right_center(gripper_x - 0.08*gripper_y_axis.getX(), gripper_y - 0.08*gripper_y_axis.getY(), gripper_z - 0.08*gripper_y_axis.getZ());
	// 	std::vector<double> gripper_left_pose7d = {left_center.getX(), left_center.getY(), left_center.getZ(), temp_gripper_q.x(), temp_gripper_q.y(), temp_gripper_q.z(), temp_gripper_q.getW()};
	// 	std::vector<double> gripper_right_pose7d = {right_center.getX(), right_center.getY(), right_center.getZ(), temp_gripper_q.x(), temp_gripper_q.y(), temp_gripper_q.z(), temp_gripper_q.getW()};

		
	// 	Eigen::Matrix4f gripper_left_pose = Eigen::Matrix4f::Zero(4,4);
	// 	Eigen::Matrix4f gripper_right_pose = Eigen::Matrix4f::Zero(4,4);
	// 	Eigen::Matrix4f world_to_gripper_left_pose = Eigen::Matrix4f::Zero(4,4);
	// 	Eigen::Matrix4f world_to_gripper_right_pose = Eigen::Matrix4f::Zero(4,4);
	// 	toTransformationMatrix(gripper_left_pose, gripper_left_pose7d);
	// 	toTransformationMatrix(gripper_right_pose, gripper_right_pose7d);

	// 	InverseTransformationMatrix(gripper_left_pose, world_to_gripper_left_pose);
	// 	InverseTransformationMatrix(gripper_right_pose, world_to_gripper_right_pose);

	// 	pcl::transformPointCloud(*objCloudWorld, *objCloudLeftGripper, world_to_gripper_left_pose);
	// 	pcl::transformPointCloud(*objCloudWorld, *objCloudRightGripper, world_to_gripper_right_pose);
			 

	// 	double local_x_max, local_x_min, local_y_max, local_y_min, local_z_max, local_z_min;
	// 	left_num = 0;
	// 	right_num = 0;
	// 	local_x_max = 0.03;
	// 	local_x_min = -0.03;
	// 	local_y_max = 0.005;
	// 	local_y_min = -0.005;
	// 	local_z_max = 0.01;
	// 	local_z_min = -0.01;
	// 	double point_x, point_y, point_z;
	// 	for(int ii=0;ii<objCloudLeftGripper->points.size();ii++) {
	// 		//std::cout << objCloudLeftGripper->points[ii].x << " " << objCloudLeftGripper->points[ii].y << objCloudLeftGripper->points[ii].z;	
	// 		point_x = objCloudLeftGripper->points[ii].x;
	// 		point_y = objCloudLeftGripper->points[ii].y;
	// 		point_z = objCloudLeftGripper->points[ii].z;
	// 		if(point_x > local_x_min && point_x < local_x_max 
	// 			&& point_y > local_y_min && point_y < local_y_max 
	// 			&& point_z > local_z_min && point_z < local_z_max){
	// 			left_num ++;
	// 			ROS_WARN_STREAM(i<<" left_num:"<<left_num);
	// 		}


	// 	}

	// 	for(int ii=0;ii<objCloudRightGripper->points.size();ii++) {
	// 		//std::cout << objCloudRightGripper->points[ii].x << " " << objCloudRightGripper->points[ii].y << objCloudRightGripper->points[ii].z;	
	// 		point_x = objCloudRightGripper->points[ii].x;
	// 		point_y = objCloudRightGripper->points[ii].y;
	// 		point_z = objCloudRightGripper->points[ii].z;
	// 		if(point_x > local_x_min && point_x < local_x_max 
	// 			&& point_y > local_y_min && point_y < local_y_max 
	// 			&& point_z > local_z_min && point_z < local_z_max){
	// 			right_num ++;
	// 			ROS_WARN_STREAM(i<<" right_num:"<<right_num);

	// 		}

	// 	}
	// 	ROS_WARN_STREAM("this pose has "<<right_num+left_num<<"points inside");
	// 	rank.push_back(left_num+right_num);
	// 	grasp_list.push_back(temp_grasp_pose);
	// }

	// for(int j = 0; j < 12; j++){
	// 	int least_index = 0;
	// 	int least = INT_MAX;
	// 	for(int k = 0; k < 12;k ++){
	// 		if(rank[k] < least && rank[k] > 0){
	// 			least = rank[k];
	// 			least_index = k;
	// 		}
	// 	}
	// 	res.return_poses.push_back(grasp_list[least_index]);  
	// 	res.grasp_rank.push_back(least);
	// 	rank[least_index] = -1;
	// }
	ROS_WARN_STREAM("collision_num:"<<collision_points_num);
	res.collision_num = collision_points_num;
	
	

	
	return true;
}
	







/*call back function for the whole grasp planning pipe line*/
bool KukaGrasp::graspSuccessServiceCB(grasping::GraspSuccess::Request &req, grasping::GraspSuccess::Response &res)
{	
	ROS_WARN("###################GRASP SUCCESS SERVICE CALL#####################");
	ROS_WARN("###################GRASP SUCCESS SERVICE CALL#####################");
	ROS_WARN("###################GRASP SUCCESS SERVICE CALL#####################");
	ROS_WARN("###################GRASP SUCCESS SERVICE CALL#####################");
	double detected_length = 0.15;
	ros::NodeHandle nh_1;
	char cam_intrinsic_topic[100];
	char cam_pose_topic[100];
	sensor_msgs::Image::ConstPtr msg_depth;
	std::string resources_path = ros::package::getPath("chimp_resources");
	system(("rosparam load " +resources_path + "/config/bin_config.yaml").c_str());
	char depth_image_topic_param[100];
	int binId = req.binId;
	XmlRpc::XmlRpcValue camIntr;
	std::vector<double> camPose7D;
	std::string depth_image_topic;
	sprintf(cam_pose_topic,"/bins/bin_%d/camera_pose", binId);
	node_->getParam(cam_pose_topic, camPose7D);
	Eigen::Matrix4f camPose = Eigen::Matrix4f::Zero(4,4);
	toTransformationMatrix(camPose, camPose7D);
	std::cout << "Camera Pose: " << std::endl << camPose << std::endl;

	// Reading camera intrinsic matrix
	sprintf(cam_intrinsic_topic,"/bins/bin_%d/camera_intrinsic", binId);
	Eigen::Matrix3f camIntrinsic = Eigen::Matrix3f::Zero(3,3);
	node_->getParam(cam_intrinsic_topic, camIntr);

	for(int32_t ii = 0; ii < camIntr.size(); ii++)
		for(int32_t jj = 0; jj < camIntr[ii].size(); jj++)
			camIntrinsic(ii, jj) = static_cast<double>(camIntr[ii][jj]);
	std::cout << "Camera Intrinsics: " << std::endl << camIntrinsic << std::endl;

	sprintf(depth_image_topic_param,"/bins/bin_%d/depth_image_topic", binId);
	node_->getParam(depth_image_topic_param, depth_image_topic);

	std::cout << "Waiting for depth image on topic: " << depth_image_topic << std::endl;
	msg_depth = ros::topic::waitForMessage<sensor_msgs::Image>(depth_image_topic, nh_1);
	cv_bridge::CvImagePtr cv_ptr_depth;
	cv::Mat depth_image;
	try {
	   cv_ptr_depth = cv_bridge::toCvCopy(*msg_depth, (*msg_depth).encoding);
	   //depth_image = cv_ptr_depth->image;
	 //    cv::imwrite("/home/cm1074/Desktop/frame-000000.depth.png", cv_ptr_depth->image);
		// readDepthImage(depth_image, "/home/cm1074/Desktop/frame-000000.depth.png");
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("cv_bridge exception: %s", e.what());\
	    exit(-1);
	}

	std::cout << "got depth image" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr objCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr objCloudWorld(new pcl::PointCloud<pcl::PointXYZ>);

	convert3dUnOrganized(cv_ptr_depth->image, camIntrinsic, objCloud);
	pcl::transformPointCloud(*objCloud, *objCloudWorld, camPose);


	objCloudWorld->header.frame_id = "world";
	cloud_pub.publish(objCloudWorld);
	ros::spinOnce();


	float x = req.x;
	float y = req.y;
	float z = req.z;

	float DIM = 0.05;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud (objCloudWorld);
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (x - DIM, x + DIM);
    pass.filter (*cloud_filtered);
    pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (y - DIM, y + DIM);
    pass.filter (*cloud_filtered);
    pass.setInputCloud (cloud_filtered);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (z - DIM , z + DIM);
    pass.filter (*cloud_filtered);
	
	std::cout << "point cloud size: " << cloud_filtered->size() << std::endl;

	res.collision_num = cloud_filtered->size();

	return true;
}



















int main(int argc, char **argv)
{
	ros::init(argc, argv, "grasp_planning");
	ros::NodeHandle nh;

	KukaGrasp kuka_grasp(nh);
	// std::vector<fcl::Vector3<double>> vertices_cube_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Vector3<double>> vertices_collision_gripper_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Vector3<double>> vertices_base_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Vector3<double>> vertices_floor_ = std::vector<fcl::Vector3<double>>();
 //  std::vector<fcl::Triangle> triangles_base_ = std::vector<fcl::Triangle>();
 //  std::vector<fcl::Triangle> triangles_floor_ = std::vector<fcl::Triangle>();
 //  std::vector<fcl::Triangle> triangles_cube_ = std::vector<fcl::Triangle>();
 //  std::vector<fcl::Triangle> triangles_collision_gripper_ = std::vector<fcl::Triangle>();
	
	ros::ServiceServer grasp_planning_service = nh.advertiseService("CollisionGraspPlanning", &KukaGrasp::graspPlanningServiceCB, &kuka_grasp);
	ros::ServiceServer grasp_success_service = nh.advertiseService("GraspSuccess", &KukaGrasp::graspSuccessServiceCB, &kuka_grasp);
	ROS_INFO("Ready to plan grasp...CollisionGraspPlanning and GraspSuccess");
	ros::spin();

	return 0;
}