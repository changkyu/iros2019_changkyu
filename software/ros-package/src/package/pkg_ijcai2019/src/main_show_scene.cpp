#include <thread>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/TextureMesh.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include "camera/camera_config.hpp"
#include "camera/camera_utils.hpp"
#include "utils/utils.hpp"
#include "utils/utils_visualization.hpp"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#define BOOST_LOG_DYN_LINK 1
#include <boost/log/trivial.hpp>
#define LOG BOOST_LOG_TRIVIAL(trace)

#include "ros_markers.hpp"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace std;
using namespace pcl;
using namespace changkyu;

static bool updated;
static bool updated_crop;
static bool running;
static vector<int> vs;
static visualization::PCLVisualizer viewer;
static map<string,camerainfo_t> name2camerainfo;
static vector<string> names_cloud;

void Thread(ros::NodeHandle* nh, PointCloud<PointXYZRGB>::Ptr cloud)
{
    while(running)
    {
        if( updated==false )
        {
            cloud->clear();
            changkyu::GetInputFromCameras(*nh, *cloud, name2camerainfo);
            updated = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void ThreadCrop(ros::NodeHandle* nh, PointCloud<PointXYZRGB>::Ptr cloud)
{
    while(running)
    {
        if( updated_crop==false )
        {
            cloud->clear();
            changkyu::GetInputFromCameras(*nh, *cloud, name2camerainfo, true);
            updated_crop = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void Callback_pclkeyboard (const visualization::KeyboardEvent &event,
                           void* param)
{
    visualization::PCLVisualizer *viewer
     = static_cast<visualization::PCLVisualizer *> (param);
    if (event.keyDown())
    {
        if( event.getKeySym()=="Escape" )
        {
            running = false;            
        }
        else if( event.getKeySym()=="F5" )
        {            
        }
        else if( event.getKeySym()=="Return" )
        {
        }        
    }
}

int main(int argc, char* argv[])
{
    string fp_config;
    string fp_path;

    // ROS init
    ros::init(argc,argv,"changkyu_show_scene");    
    ros::NodeHandle nh;
    nh.getParam("/changkyu/camera_config", fp_config);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ParseParam(fp_config, name2camerainfo);
    nh.param<string>("/changkyu/pkg_ijcai2019/tabletop_kuka/path",fp_path,"");
    if( !fs::is_regular_file(fp_path) )
    {
        ROS_ERROR_STREAM("Invalid input: " << fp_path);
        return 0;
    }

    names_cloud.resize(2);
    names_cloud[0] = "cloud";
    names_cloud[1] = "cloud_over";

    vs.resize(2);    
    viewer.setWindowName("debug");
    viewer.setSize(640*2,480*2);
    viewer.setPosition(0,0);
    viewer.setCameraPosition(2,0,1,0.5,0,-0.2,0,0,1);
    for( int i=0; i<2; i++ )
    {
        viewer.createViewPort(0.0,0.5*i,1.0,0.5*(i+1),vs[i]);
        viewer.setBackgroundColor (0.2,0.2,0.2);
        viewer.addCoordinateSystem(1.0);

        PointCloud<PointXYZRGB>::Ptr cloud_cam(new PointCloud<PointXYZRGB>);
        viewer.addPointCloud(cloud_cam, names_cloud[i], vs[i]);
    }

    for( map<string,camerainfo_t>::const_iterator it = name2camerainfo.begin();
         it != name2camerainfo.end(); it++ )
    {
        utils::addWorkspace(viewer,it->second.workspace,1,1,1,it->second.name + "workspace",vs[0]);
    }
    viewer.registerKeyboardCallback(Callback_pclkeyboard, &viewer);

    const string fp_dove = "/home/cs1080/projects/ijcai2019/software/ros-package/src/package/pkg_ijcai2019/models/dove_beauty_bar/meshes/dove_beauty_bar.obj";

    pcl::TextureMesh mesh1;    
    pcl::io::loadPolygonFileOBJ (fp_dove, mesh1);
    pcl::TextureMesh mesh2;
    pcl::io::loadOBJFile (fp_dove, mesh2);
    
    pcl::TextureMesh mesh_soap = mesh1;
    mesh_soap.tex_materials = mesh2.tex_materials;

    Eigen::Quaternionf q_dove;
    q_dove = Eigen::AngleAxisf(-89.9999/ 180.0 * M_PI, Eigen::Vector3f::UnitY());

    PointCloud<PointXYZRGB> cloud_dove;
    fromPCLPointCloud2(mesh_soap.cloud, cloud_dove);
    
    YAML::Node yaml_path = YAML::LoadFile(fp_path);
    int n_objs = (yaml_path["path"][0].size() - 1)/3;
    for( int o=1; o<=n_objs; o++ )
    {
        double x   = yaml_path["path"][0][(o-1)*3+1].as<double>();
        double y   = yaml_path["path"][0][(o-1)*3+2].as<double>();
        double yaw = yaml_path["path"][0][(o-1)*3+3].as<double>();

        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
        q = q * q_dove;
        
        Eigen::Matrix3f rot;
        rot = q.toRotationMatrix();

        Eigen::Matrix4f tf;
        tf << rot(0,0), rot(0,1), rot(0,2), x,
              rot(1,0), rot(1,1), rot(1,2), y,
              rot(2,0), rot(2,1), rot(2,2), 0.035*0.5 - 0.22,
                     0,        0,        0, 1;        
        
        PointCloud<PointXYZRGB> cloud_dove_o;
        pcl::copyPointCloud(cloud_dove, cloud_dove_o);
        transformPointCloud(cloud_dove_o, cloud_dove_o, tf);
        toPCLPointCloud2(cloud_dove_o, mesh_soap.cloud);

        stringstream ss;
        ss << "obj" << o;
        viewer.addTextureMesh(mesh_soap, ss.str(), vs[0]);
    }

    vector<string> names_obj(n_objs);
    for( int o=0; o<n_objs; o++ ) names_obj[o] = "dove_beauty_bar";

    Markers markers(nh);
    tf::Quaternion q_dove2;
    q_dove2.setRPY(0, -89.9999/ 180.0 * M_PI, 0);
    markers.AddMesh( "dove_beauty_bar", q_dove2, 0.035*0.5 - 0.210 );
    markers.PublishWorkspace();
    markers.PublishObjectsPath(yaml_path["path"],names_obj);

    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr cloud_crop(new PointCloud<PointXYZRGB>);
    std::thread thread_update(Thread,&nh,cloud);
    std::thread thread_update_crop(ThreadCrop,&nh,cloud_crop);

    running = true;
    updated = false;
    updated_crop = false;
    ros::Rate r(30);
    while( ros::ok() && running )
    {
        ros::spinOnce();
        viewer.spinOnce(100);        
        
        if( updated )
        {            
            viewer.updatePointCloud(cloud,names_cloud[0]);
            updated = false;
        }
        if( updated_crop )
        {            
            viewer.updatePointCloud(cloud_crop,names_cloud[1]);
            updated_crop = false;
        }

        r.sleep();
    }    

    thread_update.join();
    thread_update_crop.join();

    return 0;
}
