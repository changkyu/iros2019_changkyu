#ifndef CHANGKYU__ROS_GAZEBO__HPP
#define CHANGKYU__ROS_GAZEBO__HPP

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_datatypes.h>

#include <MoveItNode.hpp>
#include <Grasper.hpp>

class MyGazebo
{
public:
    struct MeshMeta
    {
        std::string name;
        std::string fp_file;
        tf::Quaternion q_offset;
        double z_offset;
    };

    void AddMesh( const std::string name, const tf::Quaternion &q_offset, double z_offset )
    {
        MeshMeta meta;
        meta.name = name;
        meta.fp_file = ros::package::getPath("pkg_ijcai2019")
                       + "/models/" + name + "/model.sdf";
        meta.q_offset = q_offset;
        meta.z_offset = z_offset;
        name2mesh.insert(make_pair(name,meta));
    }

    MyGazebo(ros::NodeHandle &nh)
    {
        clt_spawn  = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
        clt_delete = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
        clt_set_state = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        clt_get_state = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        pose_default.position.x = 0.39;
        pose_default.position.y = -0.34;
        pose_default.position.z = 0.55;
        pose_default.orientation.x = 0.0;//-0.382;
        pose_default.orientation.y = 1.0;//0.924;
        pose_default.orientation.z = 0.0;
        pose_default.orientation.w = 0.0;

        //endeff_z = 0.418;
        endeff_z = 0.370;
        tf::quaternionMsgToTF(pose_default.orientation, q_default);        
    }
    ~MyGazebo(){}

    void GotoHome()
    {
        moveit_node.goto_home();
    }

    void DeleteAllModels(const std::vector<std::string> &names)
    {        
        gazebo_msgs::DeleteModel srv_del;
        for( int i=0; i<names.size(); i++ )
        {
            srv_del.request.model_name = names[i];
            clt_delete.call(srv_del);
        }
    }

    void SpawnModel(const std::string &name, const std::string &meshname, 
                    double x, double y, double yaw              )
    {
        names_model.push_back(name);
        MeshMeta &meta = name2mesh.find(meshname)->second;

        gazebo_msgs::DeleteModel srv_del;
        srv_del.request.model_name = name;
        clt_delete.call(srv_del);
       
        std::stringstream ss;
        std::ifstream ifs(meta.fp_file.c_str());
        ss << ifs.rdbuf();

        tf::Quaternion q;
        q.setEulerZYX(yaw,0,0);
        q = q * meta.q_offset;
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = meta.z_offset;
        tf::quaternionTFToMsg(q, pose.orientation);

        gazebo_msgs::SpawnModel srv;
        srv.request.model_name = name;
        srv.request.model_xml  = ss.str();
        srv.request.initial_pose = pose;        
        //srv.request.reference_frame = "world";
        clt_spawn.call(srv);
    }

    void SetObjectNames( const std::vector<std::string> &names )
    {
        names_mesh = names;
    }

    void Transition(int o, double x, double y, double yaw)
    {
        Transition( names_mesh[o-1],x,y,yaw );
    }

    void Transfer(int o_pusher, int o_target, double x, double y, double yaw)
    {
        geometry_msgs::Pose pose_curr = moveit_node.group.getCurrentPose().pose;
        double dist_goal;
        dist_goal = sqrt( (pose_curr.position.x-x)*(pose_curr.position.x-x)+
                          (pose_curr.position.y-y)*(pose_curr.position.y-y)  );

        double x_obj, y_obj, yaw_obj;
        GetObjectState(o_pusher, x_obj, y_obj, yaw_obj);

        geometry_msgs::Pose pose = GetPose(x,y,yaw);        
        moveit_node.plan_and_execute_via_waypoints(pose);
/*
        double dist = dist_goal;
        while( dist > 0.005 )
        {
            pose_curr = moveit_node.group.getCurrentPose().pose;
            double yaw_curr = yaw_obj + (yaw-yaw_obj)*(dist/dist_goal);

            SetObjectState(o_pusher,pose_curr.position.x,pose_curr.position.y,yaw_curr);

            dist = sqrt( (pose_curr.position.x-x)*(pose_curr.position.x-x)+
                         (pose_curr.position.y-y)*(pose_curr.position.y-y)  );

            std::cout << "dist: " << dist << std::endl;
            std::cout << pose_curr.position << std::endl;

            ros::Duration(0.1).sleep();
        }
        SetObjectState(o_pusher, x, y, yaw );
*/
    }

    void SetObjectState( int o, double x, double y, double yaw )
    {
        MeshMeta &meta = name2mesh.find(names_mesh[o-1])->second;

        tf::Quaternion q;
        q.setEulerZYX(yaw,0,0);
        q = q * meta.q_offset;
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = meta.z_offset;
        tf::quaternionTFToMsg(q, pose.orientation);

        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;

        gazebo_msgs::SetModelState srv;            
        srv.request.model_state.model_name = names_model[o-1];
        //srv.request.model_state.reference_frame = "world";
        srv.request.model_state.pose = pose;
        srv.request.model_state.twist = twist;

        clt_set_state.call(srv);
    }

    void GetObjectState( int o, double &x, double &y, double &yaw )
    {
        MeshMeta &meta = name2mesh.find(names_mesh[o-1])->second;

        gazebo_msgs::GetModelState srv;
        srv.request.model_name = names_model[o-1];
        clt_get_state.call(srv);

        x = srv.response.pose.position.x;
        y = srv.response.pose.position.y;

        tf::Quaternion q;        
        tf::quaternionMsgToTF(srv.response.pose.orientation, q);
        q = q * meta.q_offset.inverse();
        yaw = tf::getYaw(q);
    }

    void SetState(std::vector<std::vector<double> > &state)
    {
        int n_objs = state.size()-1;
        for( int o=1; o<=n_objs; o++ )
        {   
            double x = state[o][0];
            double y = state[o][1];
            double yaw = state[o][2];
            SetObjectState(o,x,y,yaw);
        }
    }

private:

    bool execute_trajectory(moveit::planning_interface::MoveGroup::Plan plan)
    {
        bool success = false;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_ac_("/iiwa/PositionJointInterface_trajectory_controller/follow_joint_trajectory", true);

        control_msgs::FollowJointTrajectoryActionGoal trajectory_goal;
        trajectory_ac_.waitForServer();

        trajectory_goal.goal.trajectory = plan.trajectory_.joint_trajectory;
        //trajectory_goal.goal.path_tolerance = 
        //trajectory_goal.goal.path
        trajectory_ac_.sendGoal(trajectory_goal.goal);
        success = trajectory_ac_.waitForResult();
        if(success){
            ROS_WARN_STREAM("actionlib: success");
        }else{
            ROS_WARN_STREAM("actionlib: failed");
        }
            auto result = trajectory_ac_.getResult();
            ROS_WARN_STREAM("execute_trajectory result:"<<result->error_code);
        
        return success;
    }

    geometry_msgs::Pose GetPose(double x, double y, double z, double yaw_del)
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z + endeff_z;

        tf::Quaternion q;
        q.setEulerZYX(yaw_del,0,0);
        tf::Quaternion q_next = q * q_default;
        q_next.normalize();
        tf::quaternionTFToMsg(q_next, pose.orientation);

/*
        tf::Quaternion q;
        q.setEulerZYX(yaw,0,0);
        tf::Quaternion q_curr;
        tf::quaternionMsgToTF(pose_default.orientation, q_curr);
        tf::Quaternion q_next = q * q_curr;
        q_next.normalize();
        tf::quaternionTFToMsg(q_next, pose.orientation);
*/
        return pose;
    }

    geometry_msgs::Pose GetPose(double x, double y, double yaw)
    {
        geometry_msgs::Pose pose_curr = moveit_node.group.getCurrentPose().pose;
        return GetPose(x,y,pose_curr.position.z - endeff_z, yaw);
    }

    void Transition(double x, double y, double z, double yaw)
    {
        geometry_msgs::Pose pose_curr = moveit_node.group.getCurrentPose().pose;        
        pose_curr.orientation = pose_default.orientation;        
        pose_curr.position.z += 0.100;

        ros::Duration(1).sleep();


        std::cout << "goes up" << std::endl;
        moveit_node.plan_and_execute_via_waypoints(pose_curr);

        std::cout << "goes side" << std::endl;
        geometry_msgs::Pose pose = GetPose(x,y,z,0);
        pose.orientation = pose_default.orientation;
        pose.position.z += 0.100;
        moveit_node.plan_and_execute_via_waypoints(pose);

        std::cout << "goes down" << std::endl;
        pose.position.z -= 0.100;
        moveit_node.plan_and_execute_via_waypoints(pose);

        ros::Duration(1).sleep();
    }

    void Transition(const std::string &name, double x, double y, double yaw)
    {
        MeshMeta &meta = name2mesh.find(name)->second;
        Transition(x,y,meta.z_offset,yaw);
    }

    void GraspClose()
    {
        moveit_node.plan_and_execute_via_waypoints(0,0,-0.015);
        grasp_node.publish_command("c");
        ros::Duration(1).sleep();
    }

    void GraspOpen()
    {
        grasp_node.publish_command("o");
        moveit_node.plan_and_execute_via_waypoints(0,0,0.015);
        ros::Duration(1).sleep();
    }

private:

    ros::ServiceClient clt_spawn;
    ros::ServiceClient clt_delete;
    ros::ServiceClient clt_set_state;
    ros::ServiceClient clt_get_state;

    geometry_msgs::Pose pose_default;

    MoveItNode moveit_node;
    GraspNode  grasp_node;

    std::map<std::string,MeshMeta> name2mesh;
    std::vector<std::string> names_mesh;
    std::vector<std::string> names_model;

    double endeff_z;
    tf::Quaternion q_default;

};


#endif