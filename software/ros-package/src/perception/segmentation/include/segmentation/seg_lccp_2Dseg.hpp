#ifndef SEG_LCCP_2DSEG__H__
#define SEG_LCCP_2DSEG__H__

#include <map>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <opencv2/core/core.hpp>

#include "segmentation/seg_param.hpp"
#include "rl_msgs/SegmentationScene.h"

void ComputeConvexities(
std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> &supervoxel_clusters,
std::map<uint32_t, float>                               &supervoxel_confidences,
std::multimap<uint32_t, uint32_t>                         &supervoxel_adjacency,    
std::map<std::pair<uint32_t,uint32_t>, float>             &weights_adjacency,
uint32_t                                                  num_of_hop         );

void DoSupervoxel_from_2Dseg(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_input,
    cv::Mat &img_seg, cv::Mat &camera_K, 
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> &l2sv,
    std::map<uint32_t, float> &l2cf,
    std::multimap<uint32_t,uint32_t> &edges
);

void WeightStronglyConnectedEdges( cv::Mat &image_seg, 
    std::map<uint32_t,pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> &supervoxel_clusters,
    std::multimap<uint32_t,uint32_t> &supervoxel_adjacency,    
    cv::Mat &camera_K,
    std::map<std::pair<uint32_t,uint32_t>, float> &weights_adjacency,
    std::multimap<uint32_t,uint32_t> *edges_strong=NULL           );

void SpectralClustering(
    std::map<uint32_t,pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> &supervoxel_clusters,
    std::map<std::pair<uint32_t,uint32_t>, float> &weights_adjacency,
    std::map<uint32_t,std::set<uint32_t> > &segs_labels_out,
    std::map<uint32_t,pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> &supervoxel_out
    );

void GetSegmentationSupervoxels(
    std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> &map_sv, 
    std::map<uint32_t, std::set<uint32_t> > segs_labels,
    cv::Mat &camera_K,
    rl_msgs::SegmentationScene &scene
);

typedef class SegLCCP2DSeg
{
public:
    SegLCCP2DSeg();
    SegLCCP2DSeg(ros::NodeHandle* nh, std::string SUB_COB="/segmentation/cob");    
    ~SegLCCP2DSeg(){};

    typedef std::map<uint32_t, pcl::Supervoxel<pcl::PointXYZRGB>::Ptr> SUPERVOXELS;
    typedef std::map<std::pair<uint32_t,uint32_t>, float>              WEIGHTED_EDGES;    

    bool Segment( rl_msgs::SegmentationScene* segscene_out,
                  sensor_msgs::Image &msg_img,
                  sensor_msgs::Image &msg_dep,
                  const float depth_scale,                           
                  const std::vector<float> &camera_K,
                  const std::vector<float> &camera_RT=std::vector<float>(),
                  const std::string &param_str=""          );

    bool Segment( rl_msgs::SegmentationScene* segscene_out,
                  sensor_msgs::Image &image_input,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input, 
                  const std::string &param="",
                  const std::vector<float> &camera_K=std::vector<float>(),
                  const std::vector<float> &camera_RT=std::vector<float>() );

    bool Segment( cv::Mat &image_seg,
                  const cv::Mat &image,
                  const cv::Mat &depth,
                  const pcl::PointCloud<pcl::PointXYZRGB> &cloud_input, 
                  const std::string &param_str="",
                  const std::vector<float> &camera_K=std::vector<float>(),
                  const std::vector<float> &camera_RT=std::vector<float>(),
                  const float depth_scale = 0.001 );

private:

    bool Segment( LCCP2DSegParam &param,
                  SUPERVOXELS    &supervoxel_clusters,
                  WEIGHTED_EDGES &weights_adjacency,
                  std::map<uint32_t,std::set<uint32_t> > &segs_labels,
                  cv::Mat                                &image_seg,
                  cv::Mat                                &image_sv,
                  std::vector<shape_msgs::Plane>         &planes_bg,
                  const cv::Mat &image, 
                  const pcl::PointCloud<pcl::PointXYZRGB> &cloud_input_,
                  const std::string &param_str="",
                  const std::vector<float> &camera_K=std::vector<float>(),
                  const std::vector<float> &camera_RT=std::vector<float>() );

    ros::ServiceClient clt_cob;
    uint32_t idx_save;
} SegLCCP2DSeg;

#endif