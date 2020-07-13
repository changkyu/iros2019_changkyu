/*
File: Grasper.cpp

Authors: Aravind Sivaramakrishnan

Description: The GraspNode class takes care of the entire grasping pipeline.

Comments/TODO:
- The gripper active check if probably not comprehensive enough, and may be abstracted away.
(probably expose it as a ros service)
- Let's trust the gripper to perfectly execute its actions for now...
- setGraspWidth()
- The grasp/pre-grasp calculations are currently made only for overhead grasps, since it is
non-trivial to obtain the inverse of the approach direction directly from the quaternion.
- In GRASP state, have to call the change velocity service. Right now, this throws an error when ext
torque > 10.
*/
#include <Grasper.hpp>

GraspNode::GraspNode()
{
	// Initialize the gripper command publisher.
	gripper_command_publisher = node_handle.advertise<std_msgs::String>("/GripperCommand",1);
	gripper_opening = 63;
	// #### ROBOTIQ
	gripper_force_feedback = true;
	gripper_force_feedback_sub = node_handle.subscribe("/GripperForceFeedback", 1, &GraspNode::GripperForceFeedbackCB, this);
	std::cout<<"Created a subscriber...\n";
}

void GraspNode::GripperForceFeedbackCB(const std_msgs::String::ConstPtr& msg)
{
	#ifdef REFLEX
	std::cout<<"Callback called..."<<msg->data.c_str()<<"\n";
	// std::string feedback = msg->data.c_str();
	if(feedback == "true")
	{
		// std::cout<<"++++FF "<<msg->data.c_str()<<"  ";
		gripper_force_feedback = true;
	}
	else
	{
		// std::cout<<"----FF "<<msg->data.c_str();
		gripper_force_feedback = false;
	}
	#endif
	#ifndef REFLEX
	gripper_force_feedback = true;
	#endif
}

bool GraspNode::grasp(geometry_msgs::PoseStamped point, geometry_msgs::PoseStamped place_pose,double size, MoveItNode& moveit_node, iiwa_ros::iiwaRos& my_iiwa)
{
	exit(1);
	// setGraspPoses(point);
	// double size_ = 80;
	// setGraspWidth(size);

	// current_state = START;

	// while (ros::ok())
	// {
	// 	ROS_INFO_STREAM("Current state at Grasper: " << get_state(current_state));
	// 	if (current_state == START)
	// 	{
	// 		// /* Close the fingers and approach the pre-grasp position. */
	// 		// gripper_command.data = "c";
	// 		// gripper_command_publisher.publish(gripper_command);
	// 		// ros::Duration(0.5).sleep();
	// 		// // my_iiwa.getPathParametersService().setJointRelativeVelocity(0.5);
	// 		current_state = PREGRASP;
	// 	}
	// 	else if (current_state == PREGRASP)
	// 	{
	// 		ROS_WARN_STREAM("pre_grasp_point:"<<pre_grasp_point.pose.position.x<<","<<pre_grasp_point.pose.position.y<<","<<pre_grasp_point.pose.position.z<<","<<pre_grasp_point.pose.orientation.x<<","<<pre_grasp_point.pose.orientation.y<<","<<pre_grasp_point.pose.orientation.z<<","<<pre_grasp_point.pose.orientation.w);
	// 		/* Reach for the pre-grasp position. Open the fingers. */
	// 		// planning_success = moveit_node.plan_and_execute(pre_grasp_point);
	// 		// if(!planning_success)
	// 		// {
	// 		// 	ROS_WARN_STREAM("GRASPER: failed to go to pre_grasp_point");
	// 		// }
	// 		// utilities::sleepForMotion(my_iiwa,2.0);
	// 		// gripper_command.data = "o";
	// 		// gripper_command_publisher.publish(gripper_command);
	// 		// ros::Duration(0.5).sleep();
	// 		// planning_success ? current_state = GRASP : current_state = FAIL;

	// 		publish_command("o");
	// 		// ros::Duration(1).sleep();

	// 		planning_success = moveit_node.plan_and_execute(pre_grasp_point);
	// 		if(!planning_success)
	// 		{
	// 			ROS_WARN_STREAM("GRASPER: failed to go to pre_grasp_point");
	// 		}
	// 		// utilities::sleepForMotion(my_iiwa,1.0);
	// 		planning_success ? current_state = GRASP : current_state = FAIL;

	// 	}
	// 	else if (current_state == GRASP)
	// 	{
	// 		ROS_WARN_STREAM("grasp_point:"<<grasp_point.pose.position.x<<","<<grasp_point.pose.position.y<<","<<grasp_point.pose.position.z<<","<<grasp_point.pose.orientation.x<<","<<grasp_point.pose.orientation.y<<","<<grasp_point.pose.orientation.z<<","<<grasp_point.pose.orientation.w);
	// 		/* Reach the grasp position. Close the fingers upto the desired levels. */
	// 		planning_success = moveit_node.plan_and_execute(grasp_point);
	// 		if(!planning_success)
	// 		{
	// 			ROS_WARN_STREAM("GRASPER: failed to go to grasp_point");
	// 		}
	// 		// planning_success = moveit_node.plan_and_execute_via_waypoints(pre_grasp_point.pose,grasp_point.pose,15);
	// 		utilities::sleepForMotion(my_iiwa,1.5);
	// 		gripper_command.data = std::to_string(graspWidth);
	// 		gripper_command_publisher.publish(gripper_command);
	// 		// ros::Duration(0.25).sleep();
	// 		// ros::spinOnce();
	// 		// ros::Duration(0.25).sleep();
	// 		// ros::Duration(1.5).sleep();
	// 		// ros::spinOnce();
	// 		// ros::Duration(1.5).sleep();
	// 		ROS_WARN_STREAM("The current gripper feedback is "<<gripper_force_feedback);
	// 		if(!gripper_force_feedback)
	// 			planning_success = false;
	// 		planning_success ? current_state = FINISH : current_state = FAIL;

	// 	}
	// 	else if (current_state == FINISH)
	// 	{ 
	// 		/* Go back to the pre grasp position. */
	// 		// pre_grasp_point.pose.position.z += 0.1;
	// 		planning_success = moveit_node.plan_and_execute(place_pose);
	// 		// planning_success = moveit_node.plan_and_execute_via_waypoints(grasp_point.pose,pre_grasp_point.pose,15);
	// 		utilities::sleepForMotion(my_iiwa,1);
	// 		return planning_success;
	// 	}
	// 	else if (current_state == FAIL)
	// 	{
	// 		return false;
	// 	}
	// 	loop_rate_->sleep();
	// }
	return false;
}

bool GraspNode::grasp(geometry_msgs::PoseStamped point,double size, MoveItNode& moveit_node, iiwa_ros::iiwaRos& my_iiwa)
{
	setGraspPoses(point);
	double size_ = 80;
	setGraspWidth(size);

	current_state = START;

	while (ros::ok())
	{
		ROS_INFO_STREAM("Current state at Grasper: " << get_state(current_state));
		if (current_state == START)
		{
			#ifdef REFLEX
			/* Close the fingers and approach the pre-grasp position. */
			gripper_command.data = "c";
			//#### ROBOTIQ
			#endif
			#ifndef REFLEX
			publish_command("o");
			#endif
			// ros::Duration(0.5).sleep();
			// my_iiwa.getPathParametersService().setJointRelativeVelocity(0.5);
			current_state = PREGRASP;
		}
		else if (current_state == PREGRASP)
		{
			
			#ifndef REFLEX
			publish_command("o");
			#endif
			// ros::Duration(0.5).sleep();
			ROS_WARN_STREAM("pre_grasp_point:"<<pre_grasp_point.pose.position.x<<","<<pre_grasp_point.pose.position.y<<","<<pre_grasp_point.pose.position.z<<","<<pre_grasp_point.pose.orientation.x<<","<<pre_grasp_point.pose.orientation.y<<","<<pre_grasp_point.pose.orientation.z<<","<<pre_grasp_point.pose.orientation.w);
			planning_success = moveit_node.plan_and_execute_via_waypoints(pre_grasp_point);
			// planning_success = moveit_node.plan_and_execute(pre_grasp_point);
			
			// utilities::sleepForMotion(my_iiwa,2.0);
			planning_success ? current_state = GRASP : current_state = FAIL;
			if(!planning_success)
				grasping_failure_mode = PREGRASP_MP;

		}
		else if (current_state == GRASP)
		{
			#ifndef REFLEX
			publish_command("c");
			#endif
			/* Reach the grasp position. Close the fingers upto the desired levels. */
			ROS_WARN_STREAM("grasp_point:"<<grasp_point.pose.position.x<<","<<grasp_point.pose.position.y<<","<<grasp_point.pose.position.z<<","<<grasp_point.pose.orientation.x<<","<<grasp_point.pose.orientation.y<<","<<grasp_point.pose.orientation.z<<","<<grasp_point.pose.orientation.w);
			planning_success = moveit_node.plan_and_execute_via_waypoints(grasp_point);
			// planning_success = moveit_node.plan_and_execute(grasp_point);
			

			// utilities::sleepForMotion(my_iiwa,2.0);

			if(planning_success)
			{
				#ifdef REFLEX
				gripper_command.data = std::to_string(graspWidth);
				//#### ROBOTIQ
				gripper_command_publisher.publish(gripper_command);
				publish_command("c");
				#endif

				// ros::Duration(1.5).sleep();
				// ros::spinOnce();
				// ros::Duration(1.5).sleep();
				// ros::spinOnce();
				// ros::Duration(1.0).sleep();
				// ros::spinOnce();
				// ros::Duration(1.0).sleep();
				// ros::spinOnce();
				// ros::Duration(1.5).sleep();
				ROS_WARN_STREAM("The current gripper feedback is "<<gripper_force_feedback);
				if(!gripper_force_feedback)
				{
					grasping_failure_mode = FINGER_CLOSURE;
					planning_success = false;
				}
			}
			else
			{
				publish_command("o");
				grasping_failure_mode = GRASP_MP;
			}
			
			planning_success ? current_state = FINISH : current_state = FAIL;

		}
		else if (current_state == FINISH)
		{
			/* Go back to the pre grasp position. */
			// pre_grasp_point.pose.position.z += 0.1;
			planning_success = moveit_node.plan_and_execute_via_waypoints(pre_grasp_point);
			// planning_success = moveit_node.plan_and_execute_via_waypoints(grasp_point.pose,pre_grasp_point.pose,15);
			// utilities::sleepForMotion(my_iiwa,2.0);
			if(!planning_success)
			{
				grasping_failure_mode = RETRACT_MP;
			}
			else
			{
				#ifdef REFLEX
				// #### ROBOTIQ
				std::cout<<"Attempting to increase the force on the fingers to get a feedback.\n";
				gripper_command.data = "i";
				gripper_command_publisher.publish(gripper_command);
				ros::Duration(1.5).sleep();
				std::cout<<"Attempting to increase the force on the fingers to get a feedback.\n";				
				gripper_command.data = "i";
				gripper_command_publisher.publish(gripper_command);
				ros::Duration(1.5).sleep();
				std::cout<<"Attempting to increase the force on the fingers to get a feedback.\n";

				std::cout<<"Attempting to increase the force on the fingers to get a feedback.\n";
				gripper_command.data = "i";
				gripper_command_publisher.publish(gripper_command);
				ros::Duration(1.5).sleep();
				std::cout<<"Attempting to increase the force on the fingers to get a feedback.\n";				
				gripper_command.data = "i";
				gripper_command_publisher.publish(gripper_command);
				ros::Duration(1.5).sleep();
				std::cout<<"Attempting to increase the force on the fingers to get a feedback.\n";

				std::cout<<"Attempting to increase the force on the fingers to get a feedback.\n";
				gripper_command.data = "i";
				gripper_command_publisher.publish(gripper_command);
				ros::Duration(1.5).sleep();
				std::cout<<"Attempting to increase the force on the fingers to get a feedback.\n";				
				gripper_command.data = "i";
				gripper_command_publisher.publish(gripper_command);
				ros::Duration(1.5).sleep();
				std::cout<<"Attempting to increase the force on the fingers to get a feedback.\n";


				std::cout<<"Feedback now is.."<<gripper_force_feedback<<"\n";
				// ros::spinOnce();
				// ros::Duration(1.5).sleep();
				ROS_WARN_STREAM("The current gripper feedback is "<<gripper_force_feedback);
				ROS_WARN_STREAM("The current gripper feedback is "<<gripper_force_feedback);

				#endif

				if(!gripper_force_feedback)
				{
					grasping_failure_mode = FINGER_CLOSURE;
					planning_success = false;
				}
				else
				{
					grasping_failure_mode = SUCCESS;
					#ifdef REFLEX			
					gripper_command.data = "d";
					gripper_command_publisher.publish(gripper_command);
					#endif
				}
			}

			return planning_success;
		}
		else if (current_state == FAIL)
		{
			return false;
		}
		loop_rate_->sleep();
	}
}

/* point encapsulates the position of center of the palm and the
desired orientation of the end-effector */

void GraspNode::setGraspPoses(geometry_msgs::PoseStamped point)
{	

	// Eigen::Matrix3f rotMat;
	// Eigen::Quaternionf q;
	// q.w() = point.pose.orientation.w;
	// q.x() = point.pose.orientation.x;
	// q.y() = point.pose.orientation.y;
	// q.z() = point.pose.orientation.z;

	// rotMat = q.toRotationMatrix();
	// grasp_point = point;
	// // equivalent to grasp_point.z += 0.225
	// grasp_point.pose.position.x -= (error_margin + gripper_offset)*rotMat(0,2);
	// grasp_point.pose.position.y -= (error_margin + gripper_offset)*rotMat(1,2);
	// grasp_point.pose.position.z -= (error_margin + gripper_offset)*rotMat(2,2);

	// pre_grasp_point = grasp_point;
	// pre_grasp_point.pose.position.x -= (pre_grasp_height)*rotMat(0,2);
	// pre_grasp_point.pose.position.y -= (pre_grasp_height)*rotMat(1,2);
	// pre_grasp_point.pose.position.z -= (pre_grasp_height)*rotMat(2,2);


	grasp_point = offsetPoseInLocalFrame(point, gripper_offsets);

	pre_grasp_point = offsetPoseInLocalFrame(grasp_point, {0,0,-pre_grasp_height});

}


geometry_msgs::PoseStamped GraspNode::offsetPoseInLocalFrame(geometry_msgs::PoseStamped point, std::vector<double> local_xyz_offsets)
{
	Eigen::Matrix3f rotMat;
	Eigen::Quaternionf q;
	q.w() = point.pose.orientation.w;
	q.x() = point.pose.orientation.x;
	q.y() = point.pose.orientation.y;
	q.z() = point.pose.orientation.z;

	rotMat = q.toRotationMatrix();
	geometry_msgs::PoseStamped result = point;
	for(int i=0; i<3; ++i)
	{
		result.pose.position.x += local_xyz_offsets[i]*rotMat(0,i);
		result.pose.position.y += local_xyz_offsets[i]*rotMat(1,i);
		result.pose.position.z += local_xyz_offsets[i]*rotMat(2,i);
	}
	return result;

}

void GraspNode::setGraspWidth(double size)
{
	// if (size <= 0.5)
	// 	graspWidth = 80;
	// else if (size <= 0.55)
	// 	graspWidth = 75;
	// else if (size <= 0.6) 
	// 	graspWidth = 70;
	// else if (size <= 0.7)
	// 	graspWidth = 65;
	// else
	// 	graspWidth = 63;	
	// graspWidth = 63;
	graspWidth = (int)(gripper_opening);
}

void GraspNode::publish_command(std::string command)
{
	gripper_command.data = command;
	gripper_command_publisher.publish(gripper_command);
}

std::string GraspNode::get_state(State current_state)
{
	switch(current_state)
	{
		case START: return "START";
		case PREGRASP: return "PRE GRASP";
		case GRASP: return "GRASP";
		case FINISH: return "FINISH";
		case FAIL: return "FAIL";
		default: return "Bad state!";
	}
}