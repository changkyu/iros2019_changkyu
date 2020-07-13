#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

#include "segmentation/seg_preprocess.hpp"

using namespace std;
using namespace pcl;

void RemoveInvalidPoints(PointCloud<PointXYZRGB>::Ptr input,
                         PointCloud<PointXYZRGB>::Ptr output)
{
    // Remove Invalid Points    
    for( PointCloud<PointXYZRGB>::iterator 
         it_pt = input->begin(); it_pt != input->end(); it_pt++ )
    {        
        if( it_pt->z > 0.1 ) output->push_back(*it_pt);
    }
}

#define RANGE_ERASE (0.01) // 1cm
#define MAXITERS_BGRANSAC (1000)
#define NUM_OF_BGPLANE (1)
void RemoveBackground(PointCloud<PointXYZRGB>::Ptr cloud_inout,
                      vector<shape_msgs::Plane> &planes_bg,
                      const vector<float> &camera_RT,
                      const int num_of_remove                   )
{
    #define COS_30 (0.86602540378443860)
    uint32_t size_org = cloud_inout->size();

    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr ids_bg (new PointIndices);
    SACSegmentation<pcl::PointXYZRGB> seg_bg;
    seg_bg.setOptimizeCoefficients (true);
    seg_bg.setModelType(pcl::SACMODEL_PLANE);
    seg_bg.setMethodType(pcl::SAC_RANSAC);
    seg_bg.setMaxIterations (MAXITERS_BGRANSAC);
    seg_bg.setDistanceThreshold (RANGE_ERASE);

    PointCloud<PointXYZRGB>::Ptr cloud_tmp(new PointCloud<PointXYZRGB>);    
    for( PointCloud<PointXYZRGB>::iterator
         it_pt=cloud_inout->begin(); it_pt!=cloud_inout->end(); it_pt++ )
    {
        if( it_pt->z > 0.1 ) cloud_tmp->push_back(*it_pt);
    }
    cloud_inout->clear();
    copyPointCloud(*cloud_tmp, *cloud_inout);

    ExtractIndices<PointXYZRGB> eifilter(true);
    for( int i=0; i<num_of_remove; i++ )
    {
        seg_bg.setInputCloud(cloud_inout);
        seg_bg.segment (*ids_bg, *coefficients);

        float norm_coeff
         = sqrt(coefficients->values[0] * coefficients->values[0] +
                coefficients->values[1] * coefficients->values[1] +
                coefficients->values[2] * coefficients->values[2]   );
        coefficients->values[0] /= norm_coeff;
        coefficients->values[1] /= norm_coeff;
        coefficients->values[2] /= norm_coeff;

        if( camera_RT.size() > 0 )
        {
            float coeff_z_in_world = camera_RT[8]  * coefficients->values[0] +
                                     camera_RT[9]  * coefficients->values[1] +
                                     camera_RT[10] * coefficients->values[2];

            if( abs(coeff_z_in_world) < COS_30 ) continue;
        }

        eifilter.setInputCloud (cloud_inout);
        eifilter.setIndices (ids_bg);
        eifilter.setNegative (true);
        eifilter.setUserFilterValue (0);
        eifilter.filterDirectly(cloud_inout);
        
        cloud_tmp->clear();
        for( PointCloud<PointXYZRGB>::iterator
             it_pt=cloud_inout->begin(); it_pt!=cloud_inout->end(); it_pt++ )
        {
            if( it_pt->z > 0.1 ) cloud_tmp->push_back(*it_pt);
        }
        cloud_inout->clear();
        copyPointCloud(*cloud_tmp, *cloud_inout);

        shape_msgs::Plane plane;
        plane.coef[0] = coefficients->values[0];
        plane.coef[1] = coefficients->values[1];
        plane.coef[2] = coefficients->values[2];
        plane.coef[3] =-coefficients->values[3];
        planes_bg.push_back(plane);
    }
}

void RemoveBackground( PointCloud<PointXYZRGB>::Ptr cloud_inout,
                       const vector<float> &workspace,
                       const vector<float> &camera_RT            )
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

    Eigen::Matrix4f tf;
    tf << camera_RT[0],  camera_RT[1],  camera_RT[2],  camera_RT[3], 
          camera_RT[4],  camera_RT[5],  camera_RT[6],  camera_RT[7], 
          camera_RT[8],  camera_RT[9],  camera_RT[10], camera_RT[11], 
          camera_RT[12], camera_RT[13], camera_RT[14], camera_RT[15];

    PointCloud<PointXYZRGB>::Ptr cloud_tmp(new PointCloud<PointXYZRGB>);
    transformPointCloud(*cloud_inout, *cloud_tmp, tf_ws.inverse()*tf);

    double x_min = -workspace[7]*0.5;
    double x_max =  workspace[7]*0.5;
    double y_min = -workspace[8]*0.5;
    double y_max =  workspace[8]*0.5;
    double z_min = -workspace[9]*0.5;
    double z_max =  workspace[9]*0.5;

    PointCloud<PointXYZRGB>::Ptr cloud_out(new PointCloud<PointXYZRGB>);    
    for( size_t p=0; p<cloud_tmp->size(); p++ )
    {
        if( x_min <= cloud_tmp->points[p].x &&
            x_max >= cloud_tmp->points[p].x &&
            y_min <= cloud_tmp->points[p].y &&
            y_max >= cloud_tmp->points[p].y &&
            z_min <= cloud_tmp->points[p].z &&
            z_max >= cloud_tmp->points[p].z    )
        {
            cloud_out->push_back(cloud_inout->points[p]);
        }        
    }
    copyPointCloud(*cloud_out, *cloud_inout);
}

void RemoveNotInPlane(PointCloud<PointXYZRGB>::Ptr cloud_inout, 
                      PointCloud<PointXYZRGB>::Ptr cloud_rm    )
{
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    PointIndices::Ptr ids_plane (new PointIndices);
    SACSegmentation<pcl::PointXYZRGB> seg_bg;
    seg_bg.setOptimizeCoefficients (true);
    seg_bg.setModelType(pcl::SACMODEL_PLANE);
    seg_bg.setMethodType(pcl::SAC_RANSAC);
    seg_bg.setMaxIterations (MAXITERS_BGRANSAC);
    seg_bg.setDistanceThreshold (RANGE_ERASE);
    seg_bg.setInputCloud(cloud_inout);
    seg_bg.segment (*ids_plane, *coefficients);
    
    cloud_rm->clear();
    PointCloud<PointXYZRGB>::Ptr cloud_tmp(new PointCloud<PointXYZRGB>);
    int idx_beg = ids_plane->indices[0];
    int idx_end = 0;
    for( int o=0; o<idx_beg; o++ )
    {
        cloud_rm->push_back((*cloud_inout)[o]);
    }
    cloud_tmp->push_back((*cloud_inout)[idx_beg]);
    for( size_t i=1; i<ids_plane->indices.size(); i++ )
    {
        idx_beg = ids_plane->indices[i-1];
        idx_end = ids_plane->indices[i];
        for( int o=idx_beg+1; o<idx_end; o++ )
        {
            cloud_rm->push_back((*cloud_inout)[o]);            
        }
        cloud_tmp->push_back((*cloud_inout)[idx_end]);
    }
    for( int o=idx_end+1; o<cloud_inout->size(); o++ )
    {
        cloud_rm->push_back((*cloud_inout)[o]);
    }

    cloud_inout->clear();
    copyPointCloud(*cloud_tmp, *cloud_inout);
}