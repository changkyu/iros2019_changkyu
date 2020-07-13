#ifndef CHANGKYU__ROS_MARKERS__HPP__
#define CHANGKYU__ROS_MARKERS__HPP__

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_ros/point_cloud.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <yaml-cpp/yaml.h>
#include <ompl/geometric/PathGeometric.h>
#include "statespace.hpp"

class Markers
{
public:
    Markers(ros::NodeHandle &nh_)
    {
        nh = nh_;
        
        pub_marker_wp = nh.advertise<visualization_msgs::MarkerArray>("/pkg_ijcai2019/markers/windowpath", 100, true);
        pub_marker_eepath = nh.advertise<visualization_msgs::MarkerArray>("/pkg_ijcai2019/markers/eepath", 100, true);
        pub_marker_objspath = nh.advertise<visualization_msgs::MarkerArray>("/pkg_ijcai2019/markers/objspath", 100, true);
        pub_marker_objs = nh.advertise<visualization_msgs::MarkerArray>("/pkg_ijcai2019/markers/objs", 100, true);
        pub_marker_ws = nh.advertise<visualization_msgs::MarkerArray>("/pkg_ijcai2019/markers/workspace", 100, true);
        pub_marker_cloud = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/pkg_ijcai2019/markers/pointcloud", 100, true);
    }

    ~Markers(){}

    struct MeshMeta
    {
        std::string name;
        std::string resource;
        tf::Quaternion q_offset;
        double z_offset;
    };

    void AddMesh( const std::string name, const tf::Quaternion &q, double z_offset )
    {
        MeshMeta meta;
        meta.name = name;
        meta.resource = "package://pkg_ijcai2019/models/" + name + "/meshes/" + name + ".dae";
        meta.q_offset = q;
        meta.z_offset = z_offset;
        name2mesh.insert(make_pair(name,meta));
    }

    void PublishPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds)
    {
        int n_objs = clouds.size();
        cv::Mat colors(n_objs,1,CV_8UC3);
        for( int o=0; o<n_objs; o++ )
        {
            colors.at<cv::Vec3b>(o,0)[0] = o * 180.0 / n_objs;
            colors.at<cv::Vec3b>(o,0)[1] = 255;
            colors.at<cv::Vec3b>(o,0)[2] = 255;
        }
        cv::cvtColor(colors, colors, cv::COLOR_HSV2RGB);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZRGB>);
        for( int o=0; o<n_objs; o++ )
        {
            for( int p=0; p<clouds[o]->size(); p++ )
            {
                pcl::PointXYZRGB pt;
                pt.x = clouds[o]->points[p].x;
                pt.y = clouds[o]->points[p].y;
                pt.z = clouds[o]->points[p].z;
                pt.r = colors.at<cv::Vec3b>(o,0)[0];
                pt.g = colors.at<cv::Vec3b>(o,0)[1];
                pt.b = colors.at<cv::Vec3b>(o,0)[2];
                cloud_all->push_back(pt);
            }
        }

        cloud_all->header.frame_id = "/world";
        pub_marker_cloud.publish(cloud_all);
        ros::spinOnce();
    }

    void PublishPlane()
    {
        visualization_msgs::MarkerArray markers_ws;
        markers_ws.markers.resize(1);

        visualization_msgs::Marker &marker_5 = markers_ws.markers[0];
        marker_5.header.frame_id = "/world";
        marker_5.header.stamp = ros::Time::now();
        marker_5.ns = "table";
        marker_5.id = 0;
        marker_5.type = visualization_msgs::Marker::CUBE;
        marker_5.action = visualization_msgs::Marker::ADD;
        marker_5.scale.x = 2.00;
        marker_5.scale.y = 2.00;
        marker_5.scale.z = 0.05;
        marker_5.color.r = 0.20;
        marker_5.color.g = 0.20;
        marker_5.color.b = 0.20;
        marker_5.color.a = 0.5;
        marker_5.pose.position.x = 0.00;//0.50;
        marker_5.pose.position.y = 0.00;
        marker_5.pose.position.z = 0.00;//-marker_5.scale.z*0.5 - 0.22;
        marker_5.pose.orientation.w = 1.0;
        marker_5.lifetime = ros::Duration(10000);

        pub_marker_ws.publish(markers_ws);
        ros::spinOnce();
    }

    void PublishWorkspace()
    {
        visualization_msgs::MarkerArray markers_ws;
        markers_ws.markers.resize(5);

        visualization_msgs::Marker &marker_1 = markers_ws.markers[0];
        marker_1.header.frame_id = "/world";
        marker_1.header.stamp = ros::Time::now();
        marker_1.ns = "bound_low";
        marker_1.id = 0;
        marker_1.type = visualization_msgs::Marker::CYLINDER;
        marker_1.action = visualization_msgs::Marker::ADD;
        marker_1.scale.x = 0.40*2;
        marker_1.scale.y = 0.40*2;
        marker_1.scale.z = 0.011;
        marker_1.color.r = 1;
        marker_1.color.g = 1;
        marker_1.color.b = 1;
        marker_1.color.a = 1;
        marker_1.pose.position.z = -0.22;
        marker_1.pose.orientation.w = 1.0;
        marker_1.lifetime = ros::Duration(10000);

        visualization_msgs::Marker &marker_2 = markers_ws.markers[1];
        marker_2.header.frame_id = "/world";
        marker_2.header.stamp = ros::Time::now();
        marker_2.ns = "bound_high";
        marker_2.id = 0;
        marker_2.type = visualization_msgs::Marker::CYLINDER;
        marker_2.action = visualization_msgs::Marker::ADD;
        marker_2.scale.x = 0.70*2;
        marker_2.scale.y = 0.70*2;
        marker_2.scale.z = 0.01;
        marker_2.color.r = 0;
        marker_2.color.g = 1;
        marker_2.color.b = 0;
        marker_2.color.a = 0.5;
        marker_2.pose.position.z = -0.22;
        marker_2.pose.orientation.w = 1.0;
        marker_2.lifetime = ros::Duration(10000);

        visualization_msgs::Marker &marker_3 = markers_ws.markers[2];
        marker_3.header.frame_id = "/world";
        marker_3.header.stamp = ros::Time::now();
        marker_3.ns = "camera_1";
        marker_3.id = 0;
        marker_3.type = visualization_msgs::Marker::CYLINDER;
        marker_3.action = visualization_msgs::Marker::ADD;
        marker_3.scale.x = 0.8;
        marker_3.scale.y = 0.8;
        marker_3.scale.z = 0.01;
        marker_3.color.r = 0;
        marker_3.color.g = 0;
        marker_3.color.b = 1;
        marker_3.color.a = 0.5;
        marker_3.pose.position.x =  0.43;
        marker_3.pose.position.y = -0.34;
        marker_3.pose.position.z = -0.22;
        marker_3.pose.orientation.w = 1.0;
        marker_3.lifetime = ros::Duration(10000);

        visualization_msgs::Marker &marker_4 = markers_ws.markers[3];
        marker_4.header.frame_id = "/world";
        marker_4.header.stamp = ros::Time::now();
        marker_4.ns = "camera_2";
        marker_4.id = 0;
        marker_4.type = visualization_msgs::Marker::CYLINDER;
        marker_4.action = visualization_msgs::Marker::ADD;
        marker_4.scale.x = 0.8;
        marker_4.scale.y = 0.8;
        marker_4.scale.z = 0.01;
        marker_4.color.r = 0;
        marker_4.color.g = 0;
        marker_4.color.b = 1;
        marker_4.color.a = 0.5;
        marker_4.pose.position.x =  0.43;
        marker_4.pose.position.y =  0.34;
        marker_4.pose.position.z = -0.22;
        marker_4.pose.orientation.w = 1.0;
        marker_4.lifetime = ros::Duration(10000);

        visualization_msgs::Marker &marker_5 = markers_ws.markers[4];
        marker_5.header.frame_id = "/world";
        marker_5.header.stamp = ros::Time::now();
        marker_5.ns = "table";
        marker_5.id = 0;
        marker_5.type = visualization_msgs::Marker::CUBE;
        marker_5.action = visualization_msgs::Marker::ADD;
        marker_5.scale.x = 1.00;
        marker_5.scale.y = 2.00;
        marker_5.scale.z = 0.05;
        marker_5.color.r = 0.20;
        marker_5.color.g = 0.20;
        marker_5.color.b = 0.20;
        marker_5.color.a = 0.5;
        marker_5.pose.position.x = 0.50;
        marker_5.pose.position.y = 0.00;
        marker_5.pose.position.z = -marker_5.scale.z*0.5 - 0.22;
        marker_5.pose.orientation.w = 1.0;
        marker_5.lifetime = ros::Duration(10000);

        pub_marker_ws.publish(markers_ws);
        ros::spinOnce();
    }

    void PublishObjects( const std::vector<double> &conf,
                         const std::vector<std::string> &names  )
    {
        int n_objs = conf.size()/3;

        visualization_msgs::MarkerArray markers_objs;
        markers_objs.markers.resize(n_objs*2);

        for( int o=0; o<n_objs; o++ )
        {
            MeshMeta &meta = name2mesh.find(names[o])->second;

            std::stringstream ss;
            ss << "obj_" << o;

            double x   = conf[o*3    ];
            double y   = conf[o*3 + 1];
            double yaw = conf[o*3 + 2];
            tf::Quaternion q;
            q.setEulerZYX(yaw,0,0);
            q = q * meta.q_offset;
            q.normalize();

            geometry_msgs::Pose pose;
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = meta.z_offset;
            tf::quaternionTFToMsg(q, pose.orientation);

            visualization_msgs::Marker &marker_obj = markers_objs.markers[o];
            marker_obj.header.frame_id = "/world";
            marker_obj.header.stamp = ros::Time::now();
            marker_obj.ns = ss.str();
            marker_obj.id = 0;
            marker_obj.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker_obj.mesh_resource = meta.resource;
            marker_obj.mesh_use_embedded_materials = true;
            marker_obj.action = visualization_msgs::Marker::ADD;
            marker_obj.pose = pose;
            marker_obj.scale.x = 1;
            marker_obj.scale.y = 1;
            marker_obj.scale.z = 1;            
            marker_obj.color.a = 1;
            marker_obj.lifetime = ros::Duration(10000);

            std::stringstream ss2;
            ss2 << o+1;
            visualization_msgs::Marker &marker_idx = markers_objs.markers[o+n_objs];
            marker_idx.header.frame_id = "/world";
            marker_idx.header.stamp = ros::Time::now();
            marker_idx.ns = ss.str() + "_idx";
            marker_idx.id = 0;
            marker_idx.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_idx.text = ss2.str();
            marker_idx.scale.x = 0.01;
            marker_idx.scale.y = 0.01;
            marker_idx.scale.z = 0.01;
            marker_idx.color.r = 0.2;
            marker_idx.color.g = 0.2;
            marker_idx.color.b = 0.2;
            marker_idx.color.a = 1;
            marker_idx.pose = pose;
            marker_idx.pose.position.z += 0.02;
            marker_idx.pose.orientation.w = 1.0;
            marker_idx.lifetime = ros::Duration(10000);
        }

        pub_marker_objs.publish(markers_objs);
        ros::spinOnce();
    }

    void PublishObjectsPath( const YAML::Node node_path,
                             const std::vector<std::string> &names          )
    {
        int len_path = node_path.size();
        int n_objs = (node_path[0].size() - 1)/3;

        std::vector<std::vector<double> > vecs(len_path);

        for( int p=0; p<len_path; p++ )
        {
            vecs[p].resize((n_objs+1)*3);

            int ee = node_path[p][0].as<double>();            
            vecs[p][0] = node_path[p][(ee-1)*3 + 1].as<double>();
            vecs[p][1] = node_path[p][(ee-1)*3 + 2].as<double>();
            vecs[p][2] = node_path[p][(ee-1)*3 + 3].as<double>();
            for( int o=1; o<=n_objs; o++ )
            {
                vecs[p][o*3 + 0] = node_path[p][(o-1)*3 + 1].as<double>();
                vecs[p][o*3 + 1] = node_path[p][(o-1)*3 + 2].as<double>();
                vecs[p][o*3 + 2] = node_path[p][(o-1)*3 + 3].as<double>();
            }
        }

        PublishObjectsPath(vecs, names);
    }

    void PublishObjectsPath( const ompl::geometric::PathGeometric &path,
                             const std::vector<std::string> &names          )
    {
        int len_path = path.getStateCount();
        int n_objs = names.size();

        std::vector<std::vector<double> > vecs(len_path);

        for( int p=0; p<len_path; p++ )
        {
            vecs[p].resize((n_objs+1)*3);

            const ompl::base::State* state = path.getState(p);

            int ee = STATE_ROBOT(state);
            vecs[p][0] = STATE_OBJECT(state,ee)->getX();
            vecs[p][1] = STATE_OBJECT(state,ee)->getY();
            vecs[p][2] = STATE_OBJECT(state,ee)->getYaw();
            for( int o=1; o<=n_objs; o++ )
            {
                vecs[p][o*3 + 0] = STATE_OBJECT(state,o)->getX();
                vecs[p][o*3 + 1] = STATE_OBJECT(state,o)->getY();
                vecs[p][o*3 + 2] = STATE_OBJECT(state,o)->getYaw();
            }
        }

        PublishObjectsPath(vecs, names);
    }

    void PublishObjectsPath( const std::vector<std::vector<double> > &path,
                             const std::vector<std::string> &names          )
    {
        markers_objspath.markers.resize(1);
        markers_objspath.markers[0].action = 3;
        pub_marker_objspath.publish(markers_objspath);
        ros::spinOnce();

        int len_path = path.size();        
        int n_objs = path[0].size() / 3 - 1;        
        markers_objspath.markers.resize((len_path+1+1) * n_objs + 1);

        cv::Mat colors(n_objs,1,CV_8UC3);
        for( int o=0; o<n_objs; o++ )
        {
            colors.at<cv::Vec3b>(o,0)[0] = o * 180.0 / n_objs;
            colors.at<cv::Vec3b>(o,0)[1] = 255;
            colors.at<cv::Vec3b>(o,0)[2] = 255;
        }
        cv::cvtColor(colors, colors, cv::COLOR_HSV2RGB);


        visualization_msgs::MarkerArray markers_ee;
        markers_ee.markers.resize(len_path);
        {
            cv::Mat colors_ee(len_path,1,CV_8UC3);
            for( int p=0; p<len_path; p++ )
            {
                colors_ee.at<cv::Vec3b>(p,0)[0] = p * 180.0 / (float)len_path;
                colors_ee.at<cv::Vec3b>(p,0)[1] = 255;
                colors_ee.at<cv::Vec3b>(p,0)[2] = 255;
            }
            cv::cvtColor(colors_ee, colors_ee, cv::COLOR_HSV2RGB);

            double dist = 0;
            for( int p=0; p<len_path-1; p++ )
            {
                std::stringstream ss_ee;
                ss_ee << "ee_" << p;
                // end effector
                visualization_msgs::Marker &marker_ee = markers_ee.markers[p];                
                marker_ee.header.frame_id = "/world";
                marker_ee.header.stamp = ros::Time::now();
                marker_ee.ns = ss_ee.str();
                marker_ee.id = 0;
                marker_ee.type = visualization_msgs::Marker::ARROW;
                marker_ee.action = visualization_msgs::Marker::ADD;
                marker_ee.scale.x = 0.01;
                marker_ee.scale.y = 0.01;
                marker_ee.scale.z = 0.01;
                marker_ee.color.r = colors_ee.at<cv::Vec3b>(p,0)[0]/255.;
                marker_ee.color.g = colors_ee.at<cv::Vec3b>(p,0)[1]/255.;
                marker_ee.color.b = colors_ee.at<cv::Vec3b>(p,0)[2]/255.;
                marker_ee.color.a = 0.5;
                marker_ee.pose.orientation.w = 1.0;
                marker_ee.lifetime = ros::Duration(10000);
                geometry_msgs::Point pt1;
                pt1.x = path[p][0];
                pt1.y = path[p][1];
                pt1.z = 0 + 0.05*dist;
                marker_ee.points.push_back(pt1);
                geometry_msgs::Point pt2;
                pt2.x = path[p+1][0];
                pt2.y = path[p+1][1];
                pt2.z = 0 + 0.05*dist;
                marker_ee.points.push_back(pt2);

                dist += sqrt((pt1.x - pt2.x)*(pt1.x - pt2.x)+
                             (pt1.y - pt2.y)*(pt1.y - pt2.y));

            }
        }

        int idx;
        for( int o=0; o<n_objs; o++ )        
        {
            MeshMeta &meta = name2mesh.find(names[o])->second;

            idx = o*(len_path+2);
            std::stringstream ss;
            ss << names[o] << "_" << o << "_path";
            visualization_msgs::Marker &marker_path = markers_objspath.markers[idx];
            marker_path.header.frame_id = "/world";
            marker_path.header.stamp = ros::Time::now();
            marker_path.ns = ss.str();
            marker_path.id = 0;
            marker_path.type = visualization_msgs::Marker::LINE_STRIP;
            marker_path.action = visualization_msgs::Marker::ADD;
            marker_path.scale.x = 0.01;
            marker_path.scale.y = 0.01;
            marker_path.scale.z = 0.01;
            marker_path.color.r = colors.at<cv::Vec3b>(o,0)[0]/255.;
            marker_path.color.g = colors.at<cv::Vec3b>(o,0)[1]/255.;
            marker_path.color.b = colors.at<cv::Vec3b>(o,0)[2]/255.;
            marker_path.color.a = 0.5;
            marker_path.pose.orientation.w = 1.0;
            marker_path.lifetime = ros::Duration(10000);
            for( int p=0; p<len_path; p++ )
            {
                geometry_msgs::Point pt;
                pt.x = path[p][(o+1)*3    ];
                pt.y = path[p][(o+1)*3 + 1];
                pt.z = meta.z_offset + 0.02;
                marker_path.points.push_back(pt);
            }
            
            for( int p=0; p<len_path; p++ )
            //for( int p=len_path-1; p>=0; p-- )
            {
                idx = o*(len_path+2) + p + 2;                

                std::stringstream ss2;
                ss2 << names[o] << "_" << o << "(" << p << "/" << len_path << ")";

                double x   = path[p][(o+1)*3    ];
                double y   = path[p][(o+1)*3 + 1];
                double yaw = path[p][(o+1)*3 + 2];
                tf::Quaternion q;
                q.setEulerZYX(yaw,0,0);
                q = q * meta.q_offset;
                q.normalize();

                geometry_msgs::Pose pose;
                pose.position.x = x;
                pose.position.y = y;
                pose.position.z = meta.z_offset;
                tf::quaternionTFToMsg(q, pose.orientation);

                visualization_msgs::Marker &marker_obj = markers_objspath.markers[idx];
                marker_obj.header.frame_id = "/world";
                marker_obj.header.stamp = ros::Time::now();
                marker_obj.ns = ss2.str();
                marker_obj.id = 0;
                marker_obj.type = visualization_msgs::Marker::MESH_RESOURCE;
                marker_obj.mesh_resource = meta.resource;
                marker_obj.mesh_use_embedded_materials = (p==0||p==len_path-1);
                marker_obj.action = visualization_msgs::Marker::ADD;
                marker_obj.pose = pose;
                if( p==0 || p==len_path-1 )
                    marker_obj.pose.position.z += 0.001;
                marker_obj.scale.x = 1;
                marker_obj.scale.y = 1;
                marker_obj.scale.z = 1;
                marker_obj.color.r = colors.at<cv::Vec3b>(o,0)[0]/255.;
                marker_obj.color.g = colors.at<cv::Vec3b>(o,0)[1]/255.;
                marker_obj.color.b = colors.at<cv::Vec3b>(o,0)[2]/255.;
                marker_obj.color.a = (p==0||p==len_path-1)?1:0.5;
                marker_obj.lifetime = ros::Duration(10000);

                if( p==len_path-1 )
                {
                    idx = o*(len_path+2) + 1;            
                    ss << "index";

                    std::stringstream ss2;
                    ss2 << o+1;

                    visualization_msgs::Marker &marker_idx = markers_objspath.markers[idx];
                    marker_idx.header.frame_id = "/world";
                    marker_idx.header.stamp = ros::Time::now();
                    marker_idx.ns = ss.str();
                    marker_idx.id = 0;
                    marker_idx.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                    marker_idx.text = ss2.str();
                    marker_idx.scale.x = 0.01;
                    marker_idx.scale.y = 0.01;
                    marker_idx.scale.z = 0.01;
                    marker_idx.color.r = colors.at<cv::Vec3b>(o,0)[0]/255.;
                    marker_idx.color.g = colors.at<cv::Vec3b>(o,0)[1]/255.;
                    marker_idx.color.b = colors.at<cv::Vec3b>(o,0)[2]/255.;
                    marker_idx.color.a = 1;
                    marker_idx.pose = pose;
                    marker_idx.pose.position.z += 0.02;
                    marker_idx.pose.orientation.w = 1.0;
                    marker_idx.lifetime = ros::Duration(10000);
                }
            }
        }

        pub_marker_eepath.publish(markers_ee);
        pub_marker_objspath.publish(markers_objspath);
        ros::spinOnce();
    }

    void PublishWindowPath( const std::vector<double> &start, 
                            const std::vector<double> &goal,
                            const std::vector<std::vector<double> > &window_waypoints )
    {
        visualization_msgs::MarkerArray markers_path;
        markers_path.markers.resize(7);

        // start point
        visualization_msgs::Marker &marker_start = markers_path.markers[0];
        marker_start.header.frame_id = "/world";
        marker_start.header.stamp = ros::Time::now();
        marker_start.ns = "windowstart";
        marker_start.id = 0;
        marker_start.type = visualization_msgs::Marker::SPHERE;
        marker_start.action = visualization_msgs::Marker::ADD;
        marker_start.scale.x = 0.05;
        marker_start.scale.y = 0.05;
        marker_start.scale.z = 0.05;
        marker_start.color.r = 0;
        marker_start.color.g = 1;
        marker_start.color.b = 0;
        marker_start.color.a = 1;
        marker_start.pose.position.x = start[0];
        marker_start.pose.position.y = start[1];
        marker_start.pose.position.z = -0.175;
        marker_start.pose.orientation.x = 0;
        marker_start.pose.orientation.y = 0;
        marker_start.pose.orientation.z = 0;
        marker_start.pose.orientation.w = 1;
        marker_start.lifetime = ros::Duration(10000);

        visualization_msgs::Marker &marker_start_text = markers_path.markers[1];
        marker_start_text = marker_start;
        marker_start_text.ns = "windowstart_text";
        marker_start_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_start_text.text = "start";
        marker_start_text.pose.position.z += 0.10;

        // goal point
        // start point
        visualization_msgs::Marker &marker_goal = markers_path.markers[2];
        marker_goal.header.frame_id = "/world";
        marker_goal.header.stamp = ros::Time::now();
        marker_goal.ns = "windowgoal";
        marker_goal.id = 0;
        marker_goal.type = visualization_msgs::Marker::SPHERE;
        marker_goal.action = visualization_msgs::Marker::ADD;
        marker_goal.scale.x = 0.05;
        marker_goal.scale.y = 0.05;
        marker_goal.scale.z = 0.05;
        marker_goal.color.r = 1;
        marker_goal.color.g = 0;
        marker_goal.color.b = 0;
        marker_goal.color.a = 1;
        marker_goal.pose.position.x = goal[0];
        marker_goal.pose.position.y = goal[1];
        marker_goal.pose.position.z = -0.175;
        marker_goal.pose.orientation.x = 0;
        marker_goal.pose.orientation.y = 0;
        marker_goal.pose.orientation.z = 0;
        marker_goal.pose.orientation.w = 1;
        marker_goal.lifetime = ros::Duration(10000);

        visualization_msgs::Marker &marker_goal_text = markers_path.markers[3];
        marker_goal_text = marker_goal;
        marker_goal_text.ns = "windowgoal_text";
        marker_goal_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_goal_text.text = "goal";
        marker_goal_text.pose.position.z += 0.10;

        // path
        visualization_msgs::Marker &marker_path = markers_path.markers[4];
        marker_path.header.frame_id = "/world";
        marker_path.header.stamp = ros::Time::now();
        marker_path.ns = "windowpath";
        marker_path.id = 0;
        marker_path.type = visualization_msgs::Marker::LINE_STRIP;
        marker_path.action = visualization_msgs::Marker::ADD;
        marker_path.scale.x = 0.01;
        marker_path.scale.y = 0.01;
        marker_path.scale.z = 0.01;
        marker_path.color.r = 1;
        marker_path.color.g = 1;
        marker_path.color.b = 1;
        marker_path.color.a = 0.5;
        marker_path.pose.orientation.w = 1.0;
        marker_path.lifetime = ros::Duration(10000);

        visualization_msgs::Marker &marker_path_points = markers_path.markers[5];
        marker_path_points = marker_path;
        marker_path_points.ns = "windowpath_point";
        marker_path_points.type = visualization_msgs::Marker::POINTS;
        marker_path_points.scale.x = 0.02;
        marker_path_points.scale.y = 0.02;
        marker_path_points.scale.z = 0.02;        
        marker_path_points.color.r = 1;
        marker_path_points.color.g = 1;
        marker_path_points.color.b = 0;
        marker_path_points.color.a = 1;

        visualization_msgs::Marker &marker_window = markers_path.markers[6];
        marker_window.header.frame_id = "/world";
        marker_window.header.stamp = ros::Time::now();
        marker_window.ns = "windows";
        marker_window.id = 0;
        marker_window.type = visualization_msgs::Marker::CUBE_LIST;
        marker_window.action = visualization_msgs::Marker::ADD;
        marker_window.scale.x = 0.30;
        marker_window.scale.y = 0.30;
        marker_window.scale.z = 0.10;
        marker_window.color.r = 0;
        marker_window.color.g = 0;
        marker_window.color.b = 1;
        marker_window.color.a = 0.3;
        marker_window.pose.orientation.w = 1.0;
        marker_window.lifetime = ros::Duration(10000);

        for( int i=0; i<window_waypoints.size(); i++ )
        {
            geometry_msgs::Point p;
            p.x = window_waypoints[i][0];
            p.y = window_waypoints[i][1];
            p.z = -0.175;
            marker_path.points.push_back(p);
            marker_path_points.points.push_back(p);
            marker_window.points.push_back(p);
        }

        pub_marker_wp.publish(markers_path);
        ros::spinOnce();
    }    

private:
    ros::NodeHandle nh;
    ros::Publisher pub_marker_objs;
    ros::Publisher pub_marker_objspath;
    ros::Publisher pub_marker_eepath;
    ros::Publisher pub_marker_wp;
    ros::Publisher pub_marker_ws;
    ros::Publisher pub_marker_cloud;

    visualization_msgs::MarkerArray markers_objspath;

    std::map<std::string,MeshMeta> name2mesh;
};

#endif
