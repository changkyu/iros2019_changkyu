#include <boost/program_options.hpp>

#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>

#include "rl_msgs/seg_scene_srv.h"
#include "rl_msgs/rl_msgs_visualization.hpp"

using namespace std;
using namespace pcl;

namespace po = boost::program_options;

int main(int argc, char* argv[])
{
    bool remove_background = true;    
    string param;
    string fp_image;
    string fp_cloud;
    po::options_description desc("Example Usage");
    string camera_name;
    desc.add_options()
        ("help", "help")
        ("c,camera",  po::value<string>(&camera_name)->default_value("camera1"), "camera name")
        ("background,b", po::value<bool>(&remove_background)->default_value(true), "Remove Background?")
        ("param,p", po::value<string>(&param)->default_value(""), "Other param to pass")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help")) 
    {
        cout << desc << "\n";
        return 0;
    }

    // ROS init
    ros::init(argc,argv,"example_segmentation_lccp");
    ros::NodeHandle nh;      
    ros::ServiceClient clt_lccp_2Dseg
     = nh.serviceClient<rl_msgs::seg_scene_srv>("/changkyu/segmentation");
    
    if( param.compare("") ) param += ",";
    if( !remove_background ) param += "remove_background=0";

    rl_msgs::seg_scene_srv srv;
    srv.request.use_camera = true;
    srv.request.param = param;
    srv.request.camera_name = camera_name;

    if( clt_lccp_2Dseg.call(srv) )
    {
        int v1,v2;
        visualization::PCLVisualizer viewer;
        viewer.createViewPort(0.0,0.0,0.5,1.0,v1);
        viewer.createViewPort(0.5,0.0,1.0,1.0,v2);
        viewer.setWindowName("debug");
        viewer.setSize(960,640);
        viewer.setPosition(0,0);
        viewer.setCameraPosition(0.5,2,2,0.5,0,0,0,0,1);
        viewer.setBackgroundColor (0.2,0.2,0.2);
        viewer.addCoordinateSystem(1);
        AddSegmentationObjects(srv.response.segscene.objects, viewer,v1);
        AddSegmentationObjects2(srv.response.segscene.objects,viewer,v2);                
        viewer.spin();
    }
    else
    {
        ROS_ERROR_STREAM("Failed to do segmentation [lccp_2Dseg]");
    }

}