#include <yaml-cpp/yaml.h>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>
#define LOG BOOST_LOG_TRIVIAL(trace)

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "camera/camera_config.hpp"
#include "camera/camera_utils.hpp"

#include "planner.hpp"
#include "ros_markers.hpp"
#include "ros_kuka.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace std;
using namespace pcl;
using namespace changkyu;

bool cropPointCloud( const vector<float> &workspace,
                     const PointCloud<PointXYZRGB> &cloud_world,
                     PointCloud<PointXYZRGB> &cloud_crop )
{
    Eigen::Matrix3f tf_ws_rot
     = Eigen::Quaternionf(workspace[3],
                          workspace[4],
                          workspace[5],
                          workspace[6]).toRotationMatrix();
    Eigen::Matrix4f tf_ws;
    tf_ws << tf_ws_rot(0,0),tf_ws_rot(0,1),tf_ws_rot(0,2),workspace[0],
             tf_ws_rot(1,0),tf_ws_rot(1,1),tf_ws_rot(1,2),workspace[1],
             tf_ws_rot(2,0),tf_ws_rot(2,1),tf_ws_rot(2,2),workspace[2],
                          0,             0,             0,           1;

    PointCloud<PointXYZRGB>::Ptr cloud_tmp(new PointCloud<PointXYZRGB>);
    transformPointCloud(cloud_world, *cloud_tmp, tf_ws.inverse());

    double x_min = -workspace[7]*0.5;
    double x_max =  workspace[7]*0.5;
    double y_min = -workspace[8]*0.5;
    double y_max =  workspace[8]*0.5;
    double z_min = -workspace[9]*0.5;
    double z_max =  workspace[9]*0.5;

    PointCloud<PointXYZRGB> cloud_out;
    for( size_t p=0; p<cloud_tmp->size(); p++ )
    {
        if( x_min <= cloud_tmp->points[p].x &&
            x_max >= cloud_tmp->points[p].x &&
            y_min <= cloud_tmp->points[p].y &&
            y_max >= cloud_tmp->points[p].y &&
            z_min <= cloud_tmp->points[p].z &&
            z_max >= cloud_tmp->points[p].z    )
        {
            cloud_out.push_back(cloud_world[p]);
        }        
    }
    // cloud_cam & cloud_crop could refer the same memory space
    copyPointCloud(cloud_out, cloud_crop); 
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tabletop_kuka");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(8);
    spinner.start();

    double ee_z = 0.417;
    double plane_z = -0.21;
    string camera_name;
    string fp_path;
    map<string,camerainfo_t> name2camerainfo;
    vector<float> workspace;

    nh.getParam("/changkyu/pkg_ijcai2019/demopath_kuka/path",fp_path);
    nh.getParam("/changkyu/pkg_ijcai2019/demopath_kuka/camera_name",camera_name);
    nh.getParam("/changkyu/pkg_ijcai2019/demopath_kuka/endeff_z",ee_z);
    nh.getParam("/changkyu/pkg_ijcai2019/tabletop_kuka/crop_space",workspace);
    nh.getParam("/changkyu/pkg_ijcai2019/demopath_kuka/plane_z",plane_z);
    changkyu::ParseParam("/home/cs1080/projects/ijcai2019/software/ros-package/src/perception/camera/config/camera_info_all.yaml", name2camerainfo);
    camerainfo_t &caminfo = name2camerainfo.find(camera_name)->second;

    fs::path path_path(fp_path);
    if( !fs::is_regular_file(path_path) )
    {
        ROS_ERROR_STREAM("Invalid file: " << fp_path);
        return 0;
    }

    Markers markers(nh);
    tf::Quaternion q_dove;
    q_dove.setRPY(0, -89.9999/ 180.0 * M_PI, 0);
    markers.AddMesh( "dove_beauty_bar", q_dove, 0.035*0.5 + plane_z );

    KukaRobot kuka(nh,ee_z);
    kuka.GotoHome();

    YAML::Node node_path = YAML::LoadFile(fp_path);
    int n_objs = (node_path["path"][0].size() - 1) / 3;

    // Provide Object Information    
    vector<string> names_obj(n_objs);
    vector<RobotObjectSetup::Object> objects(n_objs);
    for( int o=0; o<n_objs; o++ )
    {
        names_obj[o] = "dove_beauty_bar";
        objects[o].name = "dove_beauty_bar";
        objects[o].dims.resize(3);
        objects[o].dims[0] = 0.035;
        objects[o].dims[1] = 0.066;
        objects[o].dims[2] = 0.096;
        objects[o].radius  = sqrt(0.096*0.096 + 0.066*0.066)*0.5;
        objects[o].q_offset.setEulerZYX(0, -89.9999/ 180.0 * M_PI, 0);
        objects[o].shape = new btBoxShape(btVector3(objects[o].dims[0]*0.5, 
                                                    objects[o].dims[1]*0.5, 
                                                    objects[o].dims[2]*0.5));
        objects[o].z_offset = 0.035 * 0.5;
        objects[o].mass = 0.135;
    }

    vector<double> conf_objs;
    for( int o=0; o<n_objs; o++ )
    {
        double x   = node_path["path"][0][o*3+1].as<double>();
        double y   = node_path["path"][0][o*3+2].as<double>();
        double yaw = node_path["path"][0][o*3+3].as<double>();
        conf_objs.push_back(x);
        conf_objs.push_back(y);
        conf_objs.push_back(yaw);
    }

    markers.PublishWorkspace();
    markers.PublishObjects(conf_objs,names_obj);

    bool repeat;
    do
    {
        repeat = false;

        sensor_msgs::Image msg_image, msg_depth;       
        PointCloud<PointXYZRGB>::Ptr cloud_cam(new PointCloud<PointXYZRGB>);    
        changkyu::GetInputFromCamera(nh, msg_image, msg_depth, *cloud_cam, caminfo);

        PointCloud<PointXYZRGB>::Ptr cloud_world(new PointCloud<PointXYZRGB>);
        transformPointCloud(*cloud_cam,*cloud_world,caminfo.tf_cam2world);

        cropPointCloud(workspace, *cloud_world, *cloud_world);

        vector<PointCloud<PointXYZRGB>::Ptr> cloud_observe(1);
        cloud_observe[0] = cloud_world;
        markers.PublishPointClouds(cloud_observe);

        std::cout << "Press \'r\' for repeat, otherwise [Enter]" << std::endl;        
        char dummy = getchar();
        if( dummy=='r' ) repeat = true;
        std::cin.ignore(INT_MAX,'\n');

    } while( repeat );

    kuka.AddMesh("dove_beauty_bar",q_dove, 0.035 + plane_z );
    kuka.SetObjectNames(names_obj);

    vector<Planner::Action> actions;
    for( int p=0; p<node_path["actions"].size(); p++ )
    {
        Planner::Action action;
        action.type        = (Planner::TYPE_ACTION)node_path["actions"][p][0].as<int>();
        action.idx_target  = node_path["actions"][p][1].as<int>();
        action.idx_target2 = node_path["actions"][p][2].as<int>();
        action.x           = node_path["actions"][p][3].as<double>();
        action.y           = node_path["actions"][p][4].as<double>();
        action.yaw         = node_path["actions"][p][5].as<double>();

        actions.push_back(action);
    }

    for( int a=0; a<actions.size(); a++ )
    {
        if( actions[a].type==Planner::ACTION_TRANSITION ) cout << "ACTION_TRANSITION: ";
        if( actions[a].type==Planner::ACTION_TRANSFER   ) cout << "ACTION_TRANSFER:   ";
        if( actions[a].type==Planner::ACTION_PUSHING    ) cout << "ACTION_PUSHING:    ";
 
        cout << actions[a].x   << ",";
        cout << actions[a].y   << ",";
        cout << actions[a].yaw << endl;
    }

    std::cout << "Ready to go [Enter]" << std::endl;
    std::cin.ignore(INT_MAX,'\n');

    for( int a=0; a<actions.size(); a++ )
    {
        if( actions[a].type==Planner::ACTION_TRANSITION )
        {       
            cout << "ACTION_TRANSITION" << ": "
                 << actions[a].x    << ", "
                 << actions[a].y    << ", "
                 << actions[a].yaw  << endl;

            kuka.Transition(actions[a].idx_target,actions[a].x,actions[a].y,actions[a].yaw);
        }
        else if( actions[a].type==Planner::ACTION_TRANSFER ||
                 actions[a].type==Planner::ACTION_PUSHING     )
        {
            vector<double> xs, ys, yaws;
            xs.push_back(actions[a-1].x);
            ys.push_back(actions[a-1].y);
            yaws.push_back(actions[a-1].yaw);
            int o = actions[a].idx_target;
            while( a<actions.size() && 
                   (actions[a].type==Planner::ACTION_TRANSFER ||
                   (actions[a].type==Planner::ACTION_PUSHING    ) ) )
            {
                cout << ((actions[a].type==Planner::ACTION_TRANSFER)?"ACTION_TRANSFER: ":"ACTION_PUSHING: ")
                     << actions[a].x    << ", "
                     << actions[a].y    << ", "
                     << actions[a].yaw  << endl;
                xs.push_back(actions[a].x);
                ys.push_back(actions[a].y);
                yaws.push_back(actions[a].yaw);
                a++;
            }
            
            kuka.GraspClose();
            kuka.Transfer(o,xs,ys,yaws,actions[a].type==Planner::ACTION_PUSHING);
            kuka.GraspOpen();
            a--;
        }
    }

    kuka.GotoHome();

}
