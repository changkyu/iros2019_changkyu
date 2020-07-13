#ifndef CHANGKYU__ROS_KUKA__HPP
#define CHANGKYU__ROS_KUKA__HPP

#include <MoveItNode.hpp>
#include <Grasper.hpp>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

class KukaRobot
{
public:    

    KukaRobot(ros::NodeHandle &n, double ee_z = 0.417)
    : nh(n)
    {
        pose_default.position.x = 0.39;
        pose_default.position.y = 0.34;
        pose_default.position.z = 0.55;
        pose_default.orientation.x = 0.0;
        pose_default.orientation.y = 1.0;
        pose_default.orientation.z = 0.0;
        pose_default.orientation.w = 0.0;

        tf::quaternionMsgToTF(pose_default.orientation, q_default);

        endeff_z = ee_z;
        safe_z = endeff_z;

        pub_marker = nh.advertise<visualization_msgs::MarkerArray>("/pkg_ijcai2019/markers/robot", 100, true);
    }
    ~KukaRobot()
    {
        GraspOpen();
        GotoHome();        
    }

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

    void SetObjectNames( const std::vector<std::string> &names )
    {
        names_obj = names;
    }

    void GotoHome()
    {
        geometry_msgs::Pose pose_curr = moveit_node.group.getCurrentPose().pose;
        if( pose_curr.position.z < safe_z )
        {
            pose_curr.position.z = safe_z;
            moveit_node.plan_and_execute_via_waypoints(pose_curr);
        } 
        moveit_node.goto_home();
    }

    void GotoDefault(double y)
    {
        geometry_msgs::Pose pose;
        pose.position.x = 0.39;
        pose.position.y = y<0?-0.34:0.34;
        pose.position.z = 0.45;
        pose.orientation = pose_default.orientation;
        moveit_node.plan_and_execute_via_waypoints(pose);        
    }

    void Transition(int o, double x, double y, double yaw)
    {
        Transition( names_obj[o-1],x,y,yaw );
    }

    /*
    void Transfer(int o, double x, double y, double yaw)
    {
        geometry_msgs::Pose pose = GetPose(names_obj[o-1],x,y,yaw);        
        ROS_INFO_STREAM("[TRANSFER] " << pose.position.x << ", " 
                                      << pose.position.y << ", " 
                                      << pose.position.z);
        PublishPlan(pose.position.x,pose.position.y,pose.position.z,true);

        pose.orientation = pose_default.orientation;
        moveit_node.plan_and_execute_via_waypoints(pose);        
    }
    */

    void Transfer(int o, const std::vector<double> &xs, 
                         const std::vector<double> &ys, 
                         const std::vector<double> &yaws,
                  bool is_pushing )
    {
        std::vector<geometry_msgs::Pose> waypoints;
        int len_path = xs.size();
        std::vector<double> zs(len_path);
 
        geometry_msgs::Pose pose_curr = moveit_node.group.getCurrentPose().pose;
        if( !moveit_node.quaternion_similarity(pose_curr,pose_default) )
        {
            pose_curr.orientation = pose_default.orientation;
            waypoints.push_back(pose_curr);
        }


        for( int i=1; i<len_path; i++ )
        {
            double vec_x = xs[i] - xs[i-1];
            double vec_y = ys[i] - ys[i-1];
            double vec_yaw = distance_angle(yaws[i],yaws[i-1]);
            double dist_pos = sqrt(vec_x*vec_x + vec_y*vec_y);

            if( dist_pos < 0.001 && abs(vec_yaw) < 0.001 )
            {
                zs[i] = zs[i-1];
                continue;
            }

            double yaw_prev = yaws[i-1];
            int n_middle = (int)(dist_pos / 0.01) + 1;
            for( int m=1; m<n_middle; m++ )
            {
                double x_mid   = xs[i-1]   + m * vec_x   / (double)n_middle;
                double y_mid   = ys[i-1]   + m * vec_y   / (double)n_middle;
                double yaw_mid = yaws[i-1] + m * vec_yaw / (double)n_middle;
                
                geometry_msgs::Pose pose = GetPose(names_obj[o-1],x_mid,y_mid, distance_angle(yaw_mid,yaws[0]));
                pose.position.z -= 0.012;

                waypoints.push_back(pose);
            }
            geometry_msgs::Pose pose_i = GetPose(names_obj[o-1],xs[i],ys[i], distance_angle(yaws[i],yaws[0]));
            pose_i.position.z -= 0.012;
            waypoints.push_back(pose_i);

            zs[i] = pose_i.position.z + 0.01;
        }

        zs[0] = zs[1];

        if( is_pushing ) PublishPlan(xs,ys,zs,1,0,1);
        else             PublishPlan(xs,ys,zs,0,0,1);

        for( int i=0; i<waypoints.size(); )
        {
            std::vector<geometry_msgs::Pose> waypoints_local;
            for( int j=0; j<20; j++ )
            {
                if( i >= waypoints.size() ) break;
                waypoints_local.push_back(waypoints[i]);
                i++;
            }            
            moveit_node.plan_and_execute_via_waypoints(waypoints_local);
        }
    }

    void GraspClose()
    {
        moveit_node.plan_and_execute_via_waypoints(0,0,-0.023);
        ros::Duration(2).sleep(); // it closes too early
        grasp_node.publish_command("c");
    }

    void GraspOpen()
    {
        ros::Duration(1).sleep(); // it opens too early
        grasp_node.publish_command("o");
        moveit_node.plan_and_execute_via_waypoints(0,0,0.023);
    }

private:

    void GetWaypoints_FixedAngle( geometry_msgs::Pose pose_init, 
                                  geometry_msgs::Pose pose_goal,
                                  std::vector<geometry_msgs::Pose> &waypoints )
    {
        double vec_x = pose_goal.position.x - pose_init.position.x;
        double vec_y = pose_goal.position.y - pose_init.position.y;
        double vec_z = pose_goal.position.z - pose_init.position.z;
        double dist_pos = sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z);
        
        int n_middle = (int)(dist_pos / 0.01) + 1;
        for( int m=1; m<n_middle; m++ )
        {
            double x_mid   = pose_init.position.x + m * vec_x / (double)n_middle;
            double y_mid   = pose_init.position.y + m * vec_y / (double)n_middle;
            double z_mid   = pose_init.position.z + m * vec_z / (double)n_middle;
           
            geometry_msgs::Pose pose;
            pose.position.x  = x_mid; 
            pose.position.y  = y_mid; 
            pose.position.z  = z_mid; 
            pose.orientation = pose_init.orientation;
            waypoints.push_back(pose);
        }
    }

    geometry_msgs::Pose GetPose(double x, double y, double z, double yaw)
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z + endeff_z;

        tf::Quaternion q;
        q.setEulerZYX(yaw,0,0);
        tf::Quaternion q_next = q * q_default;
        q_next.normalize();
        tf::quaternionTFToMsg(q_next, pose.orientation);

        return pose;
    }

    geometry_msgs::Pose GetPose(const std::string &name, double x, double y, double yaw)
    {
        MeshMeta &meta = name2mesh.find(name)->second;        
        return GetPose(x,y,meta.z_offset, yaw);
    }

    void Transition(double x, double y, double z, double yaw)
    {
        const double z_buf = 0.05;
        std::vector<geometry_msgs::Pose> waypoints;

        geometry_msgs::Pose pose_goal = GetPose(x,y,z,0);
        pose_goal.orientation = pose_default.orientation;        

        geometry_msgs::Pose pose_curr = moveit_node.group.getCurrentPose().pose;
        if( pose_curr.position.z < pose_goal.position.z + z_buf )
        {
            geometry_msgs::Pose pose_up = pose_curr;
            pose_up.position.z += z_buf;

            GetWaypoints_FixedAngle(pose_curr,pose_up,waypoints);

            pose_curr = pose_up;
        }

        if( pose_curr.position.y*y < 0 )
        {
            geometry_msgs::Pose pose_center = pose_curr;
            if(      pose_center.position.x < 0.45 )
                     pose_center.position.x = 0.45;
            else if( pose_center.position.x > 0.70 )
                     pose_center.position.x = 0.70;            
            pose_center.position.y = 0;

            GetWaypoints_FixedAngle(pose_curr,pose_center,waypoints);
            pose_curr = pose_center;
        }
        
        geometry_msgs::Pose pose_ready = pose_goal;
        pose_ready.position.z += z_buf;
        GetWaypoints_FixedAngle(pose_curr,pose_ready,waypoints);
        pose_curr = pose_ready;        
        GetWaypoints_FixedAngle(pose_curr,pose_goal,waypoints);
        pose_curr = pose_goal;   

        std::vector<double> xs, ys, zs;
        for( int i=0; i<waypoints.size(); i++ )
        {
            xs.push_back(waypoints[i].position.x);
            ys.push_back(waypoints[i].position.y);
            zs.push_back(waypoints[i].position.z);

            std::cout << waypoints[i].position.x << ", " 
                      << waypoints[i].position.y << ", " 
                      << waypoints[i].position.z << std::endl;
        }
        PublishPlan(xs,ys,zs,1,1,0);

        for( int i=0; i<waypoints.size(); )
        {
            std::vector<geometry_msgs::Pose> waypoints_local;
            for( int j=0; j<20; j++ )
            {
                if( i >= waypoints.size() ) break;
                waypoints_local.push_back(waypoints[i]);
                i++;
            }
            
            moveit_node.plan_and_execute_via_waypoints(waypoints_local);
        }
    }

    void Transition(const std::string &name, double x, double y, double yaw)
    {
        MeshMeta &meta = name2mesh.find(name)->second;
        Transition(x,y,meta.z_offset,yaw);
    }

    void PublishPlan( const std::vector<double> &xs, 
                      const std::vector<double> &ys, 
                      const std::vector<double> &zs, double r, double g, double b )
    {
        int len_path = xs.size();

        geometry_msgs::Point pt;
        geometry_msgs::Pose pose_curr = moveit_node.group.getCurrentPose().pose;
        pose_curr.position.z -= endeff_z;
        
        visualization_msgs::MarkerArray markers;
        markers.markers.resize(2);

        visualization_msgs::Marker &marker_1 = markers.markers[0];
        marker_1.header.frame_id = "/world";
        marker_1.header.stamp = ros::Time::now();
        marker_1.ns = "robot_path";
        marker_1.id = 0;
        marker_1.type = visualization_msgs::Marker::LINE_STRIP;
        marker_1.action = visualization_msgs::Marker::ADD;
        marker_1.scale.x = 0.01;
        marker_1.scale.y = 0.01;
        marker_1.scale.z = 0.01;        
        marker_1.color.r = r;
        marker_1.color.g = g;
        marker_1.color.b = b;
        marker_1.color.a = 1;
        marker_1.pose.orientation.w = 1.0;
        marker_1.lifetime = ros::Duration(10000);        
        pt.x = pose_curr.position.x;
        pt.y = pose_curr.position.y;
        pt.z = pose_curr.position.z + 0.01;
        marker_1.points.push_back(pt);
        for( int i=0; i<len_path; i++ )
        {
            pt.x = xs[i];
            pt.y = ys[i];
            pt.z = zs[i] - endeff_z + 0.01;
            marker_1.points.push_back(pt);
        }        

        visualization_msgs::Marker &marker_2 = markers.markers[1];
        marker_2.header.frame_id = "/world";
        marker_2.header.stamp = ros::Time::now();
        marker_2.ns = "robot_initgoal";
        marker_2.id = 0;
        marker_2.type = visualization_msgs::Marker::LINE_LIST;
        marker_2.action = visualization_msgs::Marker::ADD;
        marker_2.scale.x = 0.01;
        marker_2.scale.y = 0.01;
        marker_2.scale.z = 0.01;
        marker_2.color.r = 1;
        marker_2.color.g = 0;
        marker_2.color.b = 0;
        marker_2.color.a = 1;
        marker_2.pose.orientation.w = 1.0;
        marker_2.lifetime = ros::Duration(10000);        
        pt.x = pose_curr.position.x;
        pt.y = pose_curr.position.y;
        pt.z = pose_curr.position.z + 0.01;
        marker_2.points.push_back(pt);
        pt.z = pt.z + 0.02;
        marker_2.points.push_back(pt);
        pt.x = xs[len_path-1];
        pt.y = ys[len_path-1];
        pt.z = zs[len_path-1] - endeff_z + 0.01;
        marker_2.points.push_back(pt);
        pt.z = pt.z + 0.02;
        marker_2.points.push_back(pt);

        pub_marker.publish(markers);
        ros::spinOnce();
    }

    void PublishPlan(double x, double y, double z, bool is_transfer)
    {
        std::vector<double> xs(1); xs[0] = x;
        std::vector<double> ys(1); ys[0] = y;
        std::vector<double> zs(1); zs[0] = z;
        double r = 1;
        double g = is_transfer?0:1;
        double b = is_transfer?1:0;
        PublishPlan(xs,ys,zs,r,g,b);
    }

    ros::NodeHandle nh;
    ros::Publisher pub_marker;

    geometry_msgs::Pose pose_default;
    tf::Quaternion q_default;

    MoveItNode moveit_node;
    GraspNode  grasp_node;

    std::vector<std::string> names_obj;
    std::map<std::string,MeshMeta> name2mesh;

    double endeff_z;
    double safe_z;
};

#endif
