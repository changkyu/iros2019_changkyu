//////////////////////////////////////////////////////////////////////////////
// ROS
//////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

//////////////////////////////////////////////////////////////////////////////
// PCL
//////////////////////////////////////////////////////////////////////////////
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/common/pca.h>

//////////////////////////////////////////////////////////////////////////////
// OpenCV
//////////////////////////////////////////////////////////////////////////////
#include <cv_bridge/cv_bridge.h>

//////////////////////////////////////////////////////////////////////////////
// OTHER
//////////////////////////////////////////////////////////////////////////////
#include "rl_msgs/seg_scene_srv.h"

#include "segmentation/quickshift/quickshift_wrapper.hpp"
#include "segmentation/graph/spectral_clustering.hpp"
#include "segmentation/seg_param.hpp"
#include "segmentation/seg_preprocess.hpp"
#include "segmentation/seg_supervoxel.hpp"
#include "segmentation/seg_lccp_2Dseg.hpp"
#include "segmentation/vis.hpp"

#include "utils/utils.hpp"

using namespace std;
using namespace pcl;
using namespace cv;

ros::NodeHandle* nh;
string SRV_NAME;
SegLCCP2DSeg* seglccp2dseg;

typedef struct
{
    string name;
    string topic_image;
    string topic_depth;
    string topic_camerainfo;
    float depth_scale;
    vector<float> camera_RT;
    vector<float> workspace;
} camerainfo_t;
map<string,camerainfo_t> name2camerainfo;

void ParseParam(ros::NodeHandle &nh)
{
    nh.param<string>("/changkyu/segmentation/srv_name",SRV_NAME,"/changkyu/segmentation");

    name2camerainfo.clear();

    XmlRpc::XmlRpcValue camera_list;
    nh.getParam("/changkyu/camera", camera_list);
    ROS_ASSERT(camera_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for( XmlRpc::XmlRpcValue::ValueStruct::const_iterator 
         it = camera_list.begin(); it != camera_list.end(); ++it) 
    {    
        string name = it->first;
        string key_topic_image = "/changkyu/camera/"+ name +"/topic_image";
        string key_topic_depth = "/changkyu/camera/"+ name +"/topic_depth";
        string key_topic_camerainfo = "/changkyu/camera/"+ name +"/topic_image_camera_info";
        string key_depth_scale = "/changkyu/camera/"+ name +"/depth_scale";
        string key_camera_RT   = "/changkyu/camera/"+ name +"/camera_extrinsic";
        string key_workspace   = "/changkyu/camera/"+ name +"/workspace";

        camerainfo_t caminfo;
        caminfo.name = name;
        nh.getParam(key_topic_image, caminfo.topic_image);
        nh.getParam(key_topic_depth, caminfo.topic_depth);
        nh.getParam(key_topic_camerainfo, caminfo.topic_camerainfo);
        nh.getParam(key_depth_scale, caminfo.depth_scale);
        nh.getParam(key_camera_RT,   caminfo.camera_RT);
        nh.getParam(key_workspace,   caminfo.workspace);

        if( caminfo.camera_RT.size()==7 )
        {
            vector<float> pose = caminfo.camera_RT;
            Eigen::Matrix3f tf_rot
             = Eigen::Quaternionf(pose[3],
                                  pose[4],
                                  pose[5],
                                  pose[6]).toRotationMatrix();

            Eigen::Matrix4f tf_ws;
            tf_ws << tf_rot(0,0),tf_rot(0,1),tf_rot(0,2),pose[0],
                     tf_rot(1,0),tf_rot(1,1),tf_rot(1,2),pose[1],
                     tf_rot(2,0),tf_rot(2,1),tf_rot(2,2),pose[2],
                               0,          0,          0,      1;

            caminfo.camera_RT.resize(16);
            caminfo.camera_RT[0]  = tf_rot(0,0);
            caminfo.camera_RT[1]  = tf_rot(0,1);
            caminfo.camera_RT[2]  = tf_rot(0,2);
            caminfo.camera_RT[3]  = pose[0];
            caminfo.camera_RT[4]  = tf_rot(1,0);
            caminfo.camera_RT[5]  = tf_rot(1,1);
            caminfo.camera_RT[6]  = tf_rot(1,2);
            caminfo.camera_RT[7]  = pose[1];
            caminfo.camera_RT[8]  = tf_rot(2,0);
            caminfo.camera_RT[9]  = tf_rot(2,1);
            caminfo.camera_RT[10] = tf_rot(2,2);
            caminfo.camera_RT[11] = pose[2];
            caminfo.camera_RT[12] = 0;
            caminfo.camera_RT[13] = 0;
            caminfo.camera_RT[14] = 0;
            caminfo.camera_RT[15] = 1;
        }

        name2camerainfo.insert(
            pair<string,camerainfo_t>(name,caminfo) );
    }
}

bool GetInputFromCamera( sensor_msgs::Image &msg_image,
                         sensor_msgs::Image &msg_depth,                         
                         vector<float> &camera_K,
                         const string SUB_IMAGE,
                         const string SUB_DEPTH,
                         const string SUB_CAMINFO        )
{
    sensor_msgs::Image::ConstPtr img_ptr, dep_ptr;
    sensor_msgs::CameraInfo::ConstPtr ci_depth;
    try
    {        
        img_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(SUB_IMAGE, *nh, ros::Duration(3));
        dep_ptr = ros::topic::waitForMessage<sensor_msgs::Image>(SUB_DEPTH, *nh, ros::Duration(3));
        ci_depth
         = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(SUB_CAMINFO,*nh);
    }
    catch(std::exception &e)
    {
        ROS_ERROR("Exception during waitForMessage from camera: %s", e.what());
        return false;
    }

    msg_image = *img_ptr;
    msg_depth = *dep_ptr;
    camera_K.resize(9);
    for( int k=0; k<9; k++ ) camera_K[k] = ci_depth->K[k];

    return true;
}

bool Segmentation_lccp_2Dseg(rl_msgs::seg_scene_srv::Request  &req, 
                             rl_msgs::seg_scene_srv::Response &res)
{
    sensor_msgs::Image msg_img;
    sensor_msgs::Image msg_dep;
    vector<float> camera_K;
    vector<float> camera_RT;
    float depth_scale = 1.25 / 10000.0;
    string param = req.param;

    // Get Intrinsic Camera Parameters    
    if( req.use_camera )
    {
        if( req.camera_name.compare("")==0 )
        {
            ROS_ERROR_STREAM("Invalid Camera Name: " << req.camera_name);
            return false;
        } 
        
        map<string,camerainfo_t>::iterator it
         = name2camerainfo.find(req.camera_name);
        if( it == name2camerainfo.end() )
        {
            ROS_ERROR_STREAM("Unknown Camera Name: " << req.camera_name);
            return false;
        } 

        camerainfo_t &caminfo = it->second;        
        if( !GetInputFromCamera( msg_img, msg_dep, camera_K, 
                                 caminfo.topic_image, 
                                 caminfo.topic_depth,
                                 caminfo.topic_camerainfo    )) return false;

        camera_RT = caminfo.camera_RT;
        depth_scale = caminfo.depth_scale;

        if( caminfo.workspace.size() )
        {
            if(param.compare("")==0)
            {
                stringstream ss;
                ss << "workspace=";
                for( int i=0; i<caminfo.workspace.size(); i++ )
                {
                    ss << caminfo.workspace[i] << " ";
                }
                param += ss.str();
            }
        }        
    }
    else
    {
        msg_img = req.image;
        msg_dep = req.depth;
        if( req.camera_K.size()==0 || req.camera_RT.size()==0 ) return false;

        camera_K.resize(9);
        for( int k=0; k<9; k++ ) camera_K[k] = req.camera_K[k];

        camera_RT.resize(16);
        for( int rt=0; rt<12; rt++ ) camera_RT[rt] = req.camera_RT[rt];

        depth_scale = req.depth_scale;
    }

    return seglccp2dseg->Segment( &res.segscene,
                                  msg_img, msg_dep, depth_scale,
                                  camera_K, camera_RT, param );
}

int main(int argc, char* argv[])
{
    // ROS
    ros::init(argc,argv,"changkyu_segmentation_node");
    nh = new ros::NodeHandle;
    ParseParam(*nh);
    seglccp2dseg = new SegLCCP2DSeg(nh);

    ros::ServiceServer srv_seg_2Dseg
     = nh->advertiseService(SRV_NAME, Segmentation_lccp_2Dseg);

    ros::spin();

    delete seglccp2dseg;
    delete nh;

    return 0;
}
