// OpenCV
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

// Other ROS Service
#include <sensor_msgs/image_encodings.h>
#include "rl_msgs/seg_cob_srv.h"
#include "utils/utils.hpp"
#include "utils/utils_inline.hpp"

// Segmentation
#include "segmentation/meanshift/MeanShift.h"
#include "segmentation/quickshift/quickshift_wrapper.hpp"
#include "segmentation/graph/gen_graph.hpp"
#include "segmentation/graph/spectral_clustering.hpp"
#include "segmentation/seg_lccp_2Dseg.hpp"
#include "segmentation/seg_preprocess.hpp"
#include "segmentation/seg_supervoxel.hpp"

// Visualization
#include <pcl/visualization/pcl_visualizer.h>
#include "segmentation/vis.hpp"
#include "rl_msgs/rl_msgs_visualization.hpp"

using namespace std;
using namespace pcl;
using namespace cv;

template<typename PointT>
static
void Projection_K(const PointT pt, Mat &K, 
                  float *x_prj, float *y_prj)
{
    *x_prj = K.at<float>(0,0)*pt.x/pt.z + K.at<float>(0,2);
    *y_prj = K.at<float>(1,1)*pt.y/pt.z + K.at<float>(1,2);    
}

float ComputeConvexity(Supervoxel<PointXYZRGB>::Ptr sv1, 
                       Supervoxel<PointXYZRGB>::Ptr sv2)
{    
    PointXYZRGBA &pt1 = sv1->centroid_;
    Normal &nml1 = sv1->normal_;
    Eigen::Vector3f n1;
    n1 << nml1.normal_x, nml1.normal_y, nml1.normal_z;
    n1.normalize();

    PointXYZRGBA &pt2 = sv2->centroid_;
    Normal &nml2 = sv2->normal_;
    Eigen::Vector3f n2;
    n2 << nml2.normal_x, nml2.normal_y, nml2.normal_z;
    n2.normalize();

    Eigen::Vector3f vec_d; // direction x1 <- x2
    vec_d << (pt1.x-pt2.x),(pt1.y-pt2.y),(pt1.z-pt2.z);
    vec_d.normalize();

    Eigen::Vector3f vec_s = n1.cross(n2);
    vec_s.normalize();

#if 0
    // Convexity
    float convexity = 1/(1+exp(10*( -(n1-n2).dot(vec_d) -0.2)));
    // Sanity
    float sanity =    1/(1+exp(20*(abs(vec_s.dot(vec_d))-COS_45)));
#else

    float c1 = n1.dot(vec_d);
    float c2 = n2.dot(vec_d);
    float c1_c2 = c1-c2;

    if( c1_c2 < -0.2 ) // concave
    {
        return 0;
    }    
    else if( c1_c2 > 0.2 ) // convex
    {   
        if( abs(vec_s.dot(vec_d)) > COS_45 )
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }    
    else
    {
        if( n1.dot(n2) > COS_15 )
        {
            float abs_c1c2 = c1 * c2;
            if( abs_c1c2 > 0.01 )
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }
        else
        {
            #if 0
            if( abs(vec_s.dot(vec_d)) > 0.5 )
            {
                return abs(vec_s.dot(vec_d));
            }
            else
            {
                return 1.003;
            }
            #endif
            return 0;
        }
    }
#endif
}

void ComputeConvexities(
    map<uint32_t, Supervoxel<PointXYZRGB>::Ptr> &supervoxel_clusters,
    map<uint32_t,float>                         &supervoxel_confidences,
    multimap<uint32_t, uint32_t>                &supervoxel_adjacency,    
    map<pair<uint32_t,uint32_t>, float>         &weights_adjacency,    
    uint32_t                                    num_of_hop            )
{
    if( num_of_hop == 1 )
    {
        set<pair<uint32_t, uint32_t> > set_2hop;
        for(multimap<uint32_t, uint32_t>::iterator
            it_adj  = supervoxel_adjacency.begin();
            it_adj != supervoxel_adjacency.end(); it_adj++ )
        {    
            // already have it
            pair<uint32_t,uint32_t> edge_1(it_adj->first,it_adj->second);
            map<pair<uint32_t,uint32_t>, float>::iterator it_1
             = weights_adjacency.find(edge_1);
            if( it_1 == weights_adjacency.end() )
            {
                map<uint32_t,float>::iterator it_conf1
                 = supervoxel_confidences.find(it_adj->first);
                map<uint32_t,float>::iterator it_conf2
                 = supervoxel_confidences.find(it_adj->second);

                float weight = 0;
                if( it_conf1->second < 0.1 || it_conf2->second < 0.1 )
                {
                    weight = 0;
                }
                else
                {
                    weight = ComputeConvexity( 
                        supervoxel_clusters.find(it_adj->first )->second,
                        supervoxel_clusters.find(it_adj->second)->second );
//                    if( weight > 0.1 ) weight = 1;
//                    else weight = 0;
                }

                pair<uint32_t,uint32_t> edge_2(it_adj->second,it_adj->first);
                weights_adjacency.insert(
                  pair<pair<uint32_t,uint32_t>,float>(edge_1,weight));
                weights_adjacency.insert(
                  pair<pair<uint32_t,uint32_t>,float>(edge_2,weight));
            }
        }
    }
    else if( num_of_hop == 2 )
    {
        set<pair<uint32_t, uint32_t> > set_2hop;
        for(multimap<uint32_t, uint32_t>::iterator
            it_adj  = supervoxel_adjacency.begin();
            it_adj != supervoxel_adjacency.end(); it_adj++ )
        {        
            float weight_min = INFINITY;

            // convexity for 1 hop neighbors
            pair<set<pair<uint32_t,uint32_t> >::iterator,bool> ret;
            ret = set_2hop.insert(
                  pair<uint32_t,uint32_t>(it_adj->first,it_adj->second));
            if( ret.second )
            {
                float weight = ComputeConvexity(
                    supervoxel_clusters.find(it_adj->first)->second,
                    supervoxel_clusters.find(it_adj->second)->second  );
                if( weight_min > weight ) weight_min = weight;
            }
            else
            {
                cout << "["<<it_adj->first<<"]-["<<it_adj->second<<"] dup" << endl;
                continue;
            }

            // convexity for 2 hop neighbors
            pair< multimap<uint32_t,uint32_t>::iterator,
                  multimap<uint32_t,uint32_t>::iterator  > ret2
             = supervoxel_adjacency.equal_range(it_adj->second);
            for( multimap<uint32_t,uint32_t>::iterator it_2hop=ret2.first;
                 it_2hop!=ret2.second; it_2hop++ )
            {
                if( it_2hop->second != it_adj->first )
                {            
                    float weight = ComputeConvexity(
                        supervoxel_clusters.find(it_adj->first)->second,
                        supervoxel_clusters.find(it_2hop->second)->second  );
                    if( weight_min > weight ) weight_min = weight;                
                }            
            }        

            if( weight_min == INFINITY )
            {
                cout << "["<<it_adj->first<<"]-["<<it_adj->second<<"] INFINITY???" << endl;
            }

            pair<uint32_t,uint32_t> edge_1(it_adj->first,it_adj->second);
            pair<uint32_t,uint32_t> edge_2(it_adj->second,it_adj->first);
            map<pair<uint32_t,uint32_t>, float>::iterator it_2
             = weights_adjacency.find(edge_2);
            if( it_2 != weights_adjacency.end() )
            {
                if( it_2->second < weight_min ) weight_min = it_2->second;
                else                            it_2->second = weight_min;
            }
            weights_adjacency.insert(pair<pair<uint32_t,uint32_t>,float>
                                                (edge_1,weight_min));
        }
    }

    // Post-processing for isolated supervoxel
    //for( )
}

void DepthFirstLabeling(PointCloud<PointXYZRGBL> &cloud_seg,
                        int x, int y, int label_new,
                        PointCloud<PointXYZRGB> &cloud_ptr)
{    
    PointXYZRGBL* ptl = &(cloud_seg.at(x,y));
    if( ptl->label >> 31 == 0 ) return; // already labeled

    uint32_t label = ptl->label;
    ptl->label = label_new;
    
    PointXYZRGB pt;
    pt.x=ptl->x;pt.y=ptl->y;pt.z=ptl->z;pt.r=ptl->r;pt.g=ptl->g;pt.b=ptl->b;
    cloud_ptr.push_back(pt);

    // find a nearest point with same label on the right
    for( int i=1; i<10 && 0 <= x-i && x-i < cloud_seg.width && 
                          0 <= y   && y   < cloud_seg.height;   i++ )
    {
        PointXYZRGBL* ptl_next = &(cloud_seg.at(x-i,y));
        if( ptl_next->z != 0 )
        {
            if( ptl_next->label == label && 
                //geometry::distance( *ptl, *ptl_next ) <= 0.005 )
                compute_dist( *ptl, *ptl_next ) <= 0.005 )
            {
                DepthFirstLabeling(cloud_seg, x-i, y, label_new, cloud_ptr);
            }
            break;
        }
    }
    for( int i=1; i<10 && 0 <= x+i && x+i < cloud_seg.width && 
                          0 <= y   && y   < cloud_seg.height;   i++ )
    {
        PointXYZRGBL* ptl_next = &(cloud_seg.at(x+i,y));
        if( ptl_next->z != 0 ) 
        {
            if( ptl_next->label == label && 
                //geometry::distance( *ptl, *ptl_next ) <= 0.005 )
                compute_dist( *ptl, *ptl_next ) <= 0.005 )
            {
                DepthFirstLabeling(cloud_seg, x+i, y, label_new, cloud_ptr);
            }
            break;
        }
    }
    for( int i=1; i<10 && 0 <= x   && x   < cloud_seg.width && 
                          0 <= y-i && y-i < cloud_seg.height;   i++ )
    {
        PointXYZRGBL* ptl_next = &(cloud_seg.at(x,y-i));
        if( ptl_next->z != 0 ) 
        {
            if( ptl_next->label == label && 
                //geometry::distance( *ptl, *ptl_next ) <= 0.005 )
                compute_dist( *ptl, *ptl_next ) <= 0.005 )
            {
                DepthFirstLabeling(cloud_seg, x, y-i, label_new, cloud_ptr);
            }
            break;
        }
    }
    for( int i=1; i<10 && 0 <= x   && x   < cloud_seg.width && 
                          0 <= y+i && y+i < cloud_seg.height;   i++ )
    {
        PointXYZRGBL* ptl_next = &(cloud_seg.at(x,y+i));
        if( ptl_next->z != 0 )
        {
            if( ptl_next->label == label && 
                //geometry::distance( *ptl, *ptl_next ) <= 0.005 )
                compute_dist(*ptl, *ptl_next ) <= 0.005 )
            {
                DepthFirstLabeling(cloud_seg, x, y+i, label_new, cloud_ptr);
            }
            break;
        }
    }
}

void DoSupervoxel_from_2Dseg(
    PointCloud<PointXYZRGB>::Ptr &cloud_input,
    Mat &img_seg, Mat &camera_K,
    map<uint32_t, Supervoxel<PointXYZRGB>::Ptr> &l2sv_out,
    map<uint32_t, float>                        &l2cf,
    multimap<uint32_t,uint32_t> &edges
)
{    
    map<uint32_t, Supervoxel<PointXYZRGB>::Ptr> l2sv;

    int min_n_voxels = cloud_input->points.size() * 0.001;

    // initialize
    int width = img_seg.cols;
    int height = img_seg.rows;

    PointCloud<PointXYZRGBL> cloud_seg(width,height);
    for( int y=0; y<height; y++ )
    {
        for( int x=0; x<width; x++ )
        {
            PointXYZRGBL &ptl = cloud_seg.at(x,y);
            ptl.x=0; ptl.y=0; ptl.z=0; ptl.r=0; ptl.g=0; ptl.b=0; ptl.label=0;
        }
    }

    // assign labels to clouds
    // neg: pre-assigned not confirmed yet
    // 0: outlier
    // pos: already assigned
    for(size_t p=0; p<cloud_input->points.size(); p++)
    {
        PointXYZRGB &pt = cloud_input->points[p];
        if(pt.z < 0.1) continue; // skip an invalid cloud

        float x,y;
        Projection_K<PointXYZRGB>(pt,camera_K, &x,&y);
        int r=int(y+0.5), c=int(x+0.5);
        if( r<0 || height<=r || c<0 || width<=c ) continue;

        uint32_t label = img_seg.at<short int>(r,c);

        PointXYZRGBL &ptl = cloud_seg.at(c,r);
        ptl.x=pt.x; ptl.y=pt.y; ptl.z=pt.z; ptl.r=pt.r; ptl.g=pt.g; ptl.b=pt.b;

        ptl.label=-(label+1); // assign negative labels        
    }

    // Depth first labeling
    multimap<uint32_t,uint32_t> old2new;    
    int label_new = 1;
    for(size_t p=0; p<cloud_input->points.size(); p++)
    {
        PointXYZRGB &pt = cloud_input->points[p];
        if(pt.z < 0.1) continue; // skip an invalid cloud

        float x,y;
        Projection_K<PointXYZRGB>(pt,camera_K, &x,&y);
        x=int(x+0.5); y=int(y+0.5);        
        if( y<0 || height<=y || x<0 || width<=x ) continue;

        // skip already assigned label
        uint32_t label_old = cloud_seg.at(x,y).label;
        if( label_old >> 31 == 0) continue;

        // Depth first iteration
        PointCloud<PointXYZRGB> cloud_out;
        DepthFirstLabeling(cloud_seg, x, y, label_new, cloud_out);

        // SuperVoxel
        old2new.insert(pair<uint32_t,uint32_t>( label_old, label_new ));

        Supervoxel<PointXYZRGB>::Ptr sv_ptr(new Supervoxel<PointXYZRGB>());
        copyPointCloud(cloud_out, *sv_ptr->voxels_);
        l2sv.insert(
          pair<uint32_t, Supervoxel<PointXYZRGB>::Ptr>(label_new,sv_ptr));

        label_new++;
    }

    // find misaligned cloud
    PointCloud<PointXYZRGB>::Ptr cloud_inlier(new PointCloud<PointXYZRGB>());
    PointCloud<PointXYZRGB>::Ptr cloud_outlier(new PointCloud<PointXYZRGB>());
    for( multimap<uint32_t,uint32_t>::iterator it_old2new = old2new.begin();
         it_old2new != old2new.end();                                        )
    {        
        multimap<uint32_t,uint32_t>::iterator it_next
         = old2new.upper_bound(it_old2new->first);

        uint32_t label_prev=0;
        uint32_t label_inlier=0;
        vector<uint32_t> labels_outlier;
        size_t n_voxels_max = 0;
        while( it_old2new != it_next )
        {            
            uint32_t label = it_old2new->second;
            size_t n_voxels = l2sv.find(label)->second->voxels_->size();

            if( n_voxels_max < n_voxels )
            {
                if( label_prev != 0 ) labels_outlier.push_back(label_prev);
                label_prev = label;
                label_inlier = label;
                n_voxels_max = n_voxels;
            }
            else
            {
                labels_outlier.push_back(label);
            }
            it_old2new++;
        }

        map<uint32_t, Supervoxel<PointXYZRGB>::Ptr>::iterator it_l2sv
         = l2sv.find(label_inlier);
        Supervoxel<PointXYZRGB>::Ptr sv_inlier = it_l2sv->second;
        if( sv_inlier->voxels_->size() > min_n_voxels )
        {
            // Erase points not in the same plane            
//            PointCloud<PointXYZRGB>::Ptr cloud_rm(new PointCloud<PointXYZRGB>);
//            RemoveNotInPlane(sv_inlier->voxels_, cloud_rm);            
//            *cloud_outlier += *cloud_rm;
            
            *cloud_inlier += *sv_inlier->voxels_;
        }
        else
        {
            *cloud_outlier += *sv_inlier->voxels_;
            l2sv.erase(it_l2sv);
        }
        
        for( size_t i=0; i<labels_outlier.size(); i++ )
        {
            it_l2sv = l2sv.find(labels_outlier[i]);
            *cloud_outlier += *it_l2sv->second->voxels_;
            l2sv.erase(it_l2sv);
        }        

        it_old2new = it_next;
    }

    // Find the correct labels for misaligned points
    KdTreeFLANN<PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud_inlier);
    vector<int> idx(1);
    vector<float> dist(1);
    for(size_t p=0; p<cloud_outlier->size(); p++)
    {
        if( kdtree.nearestKSearch(cloud_outlier->points[p], 1, idx, dist) > 0 &&
            dist[0] < 0.001
        )
        {            
            float x,y;

            Projection_K<PointXYZRGB>(cloud_outlier->points[p],camera_K, &x,&y);
            x=int(x+0.5); y=int(y+0.5);
            PointXYZRGBL &pt_out = cloud_seg.at(x,y);

            Projection_K<PointXYZRGB>(cloud_inlier->points[idx[0]],camera_K, &x,&y);
            x=int(x+0.5); y=int(y+0.5);
            PointXYZRGBL &pt_in = cloud_seg.at(x,y);

            pt_out.label = pt_in.label;

            Supervoxel<PointXYZRGB>::Ptr sv_inlier=l2sv.find(pt_in.label)->second;
            
            PointXYZRGB pt;
            pt.x=pt_out.x; pt.y=pt_out.y; pt.z=pt_out.z;
            pt.r=pt_out.r; pt.g=pt_out.g; pt.b=pt_out.b;
            sv_inlier->voxels_->push_back(pt);
        }
        else
        {
            float x,y;

            Projection_K<PointXYZRGB>(cloud_outlier->points[p],camera_K, &x,&y);
            x=int(x+0.5); y=int(y+0.5);
            PointXYZRGBL &pt_out = cloud_seg.at(x,y);

            pt_out.x=0; pt_out.y=0; pt_out.z=0; 
            pt_out.r=0; pt_out.g=0; pt_out.b=0; 
            pt_out.label=0; 
        }
    }

    // DoSupervoxel
    label_new = 1;
    SuperVoxelParam param_sv;    
    for( int y=0; y<height; y++ )
    {
        for( int x=0; x<width; x++ )
        {
            cloud_seg.at(x,y).label = 0;
        }
    }
    for(map<uint32_t, Supervoxel<PointXYZRGB>::Ptr>::iterator
        it_l2sv = l2sv.begin(); it_l2sv != l2sv.end(); it_l2sv++)
    {
        // Supervoxel
        map<uint32_t,Supervoxel<PointXYZRGB>::Ptr> l2sv_local;
        multimap<uint32_t, uint32_t> adj_local;
        PointCloud<PointXYZL>::Ptr sv_labeled(new PointCloud<PointXYZL>);
        DoSupervoxelClustering(it_l2sv->second->voxels_, param_sv, 
                               &l2sv_local, &adj_local, &sv_labeled);

        old2new.clear();
        for( map<uint32_t,Supervoxel<PointXYZRGB>::Ptr>::iterator
             it_l2sv_local = l2sv_local.begin(); 
             it_l2sv_local!= l2sv_local.end(); it_l2sv_local++ )
        {
            Supervoxel<PointXYZRGB>::Ptr sv(new Supervoxel<PointXYZRGB>);
            sv->centroid_  = it_l2sv_local->second->centroid_;
            sv->normal_    = it_l2sv_local->second->normal_;            

            float confidence = 1;
            if( it_l2sv_local->second->normal_.normal_z > -0.25 )
            {
                confidence = 0;
            }
            l2cf.insert(pair<uint32_t,uint32_t>(label_new,confidence));

            l2sv_out.insert(pair<uint32_t,Supervoxel<PointXYZRGB>::Ptr>(
                label_new, sv));
            old2new.insert(
                pair<uint32_t,uint32_t>(it_l2sv_local->first,label_new));
            label_new++;
        }

        for( size_t p=0; p<sv_labeled->size(); p++ )
        {
            PointXYZL &ptl = (*sv_labeled)[p];
            if( ptl.z < 0.1 ) continue;

            map<uint32_t,uint32_t>::iterator it_old2new = old2new.find(ptl.label);
            if( it_old2new == old2new.end() ) continue;

            float x,y;
            Projection_K<PointXYZL>(ptl,camera_K, &x,&y);
            x=int(x+0.5); y=int(y+0.5);

            PointXYZRGBL &pt_seg = cloud_seg.at(x,y);
            if( pt_seg.z < 0.1 ) continue;

            pt_seg.label = it_old2new->second;

            PointXYZRGB pt;
            pt.x=pt_seg.x; pt.y=pt_seg.y; pt.z=pt_seg.z;
            pt.r=pt_seg.r; pt.g=pt_seg.g; pt.b=pt_seg.b;
            l2sv_out.find(pt_seg.label)->second->voxels_->push_back(pt);
        }
    }

    // Connecting Edges
    set<pair<uint32_t,uint32_t> > set_edges;
    ConnectEdges(cloud_seg, set_edges);
    for(set<pair<uint32_t,uint32_t> >::iterator it_edge = set_edges.begin();
        it_edge != set_edges.end(); it_edge++ )
    {
        if( l2sv_out.find(it_edge->first)  != l2sv_out.end() &&
            l2sv_out.find(it_edge->second) != l2sv_out.end()    )
        {
            edges.insert(pair<uint32_t,uint32_t>(it_edge->first,it_edge->second));
        }
        else
        {
//            cout << "skip edge [" << it_edge->first  << "]-[" 
//                                  << it_edge->second << "]";
        }
    }
}

void WeightStronglyConnectedEdges( Mat &image_seg, 
    map<uint32_t,Supervoxel<PointXYZRGB>::Ptr> &supervoxel_clusters,
    multimap<uint32_t,uint32_t> &supervoxel_adjacency,    
    Mat &camera_K,
    map<pair<uint32_t,uint32_t>, float> &weights_adjacency,
    multimap<uint32_t,uint32_t> *edges_strong           )
{    
    for(multimap<uint32_t, uint32_t>::iterator
        it_adj  = supervoxel_adjacency.begin();
        it_adj != supervoxel_adjacency.end();
        it_adj++
    )
    {
        float x,y;
        Projection_K<PointXYZRGBA>(
          supervoxel_clusters.find(it_adj->first)->second->centroid_,
          camera_K,&x,&y);
        uint32_t label_1=image_seg.at<short int>(int(y+0.5),int(x+0.5));
        Projection_K<PointXYZRGBA>(
          supervoxel_clusters.find(it_adj->second)->second->centroid_,
          camera_K,&x,&y);
        uint32_t label_2=image_seg.at<short int>(int(y+0.5),int(x+0.5));

        pair<uint32_t,uint32_t> edge_1(it_adj->first,it_adj->second);
        pair<uint32_t,uint32_t> edge_2(it_adj->second,it_adj->first);
        if( label_1==label_2 )
        {
            map<pair<uint32_t,uint32_t>, float>::iterator it_weight
             = weights_adjacency.find(edge_1);

            if( it_weight != weights_adjacency.end() )
            {
                //it_weight->second += 1;
                it_weight->second = 1;
            }
            else
            {
                cout << "???" << endl;
            }
            
            if( edges_strong )
            {
                // build strong edges
                bool found = false;
                pair< multimap<uint32_t,uint32_t>::iterator,
                      multimap<uint32_t,uint32_t>::iterator  > ret
                 = edges_strong->equal_range(it_adj->first);
                for( multimap<uint32_t,uint32_t>::iterator 
                     it_strong=ret.first; it_strong!=ret.second; it_strong++ )
                {
                    if( it_strong->second == it_adj->second )
                    {
                        found = true;
                        break;
                    }
                }
                if( !found )
                {
                    edges_strong->insert(
                      pair<uint32_t,uint32_t>(it_adj->first,it_adj->second));
                    edges_strong->insert(
                      pair<uint32_t,uint32_t>(it_adj->second,it_adj->first));
                }
            }
        }            
    }
}

void SpectralClustering(
    map<uint32_t, Supervoxel<PointXYZRGB>::Ptr> &supervoxel_clusters,
    map<pair<uint32_t,uint32_t>, float> &weights_adjacency,
    map<uint32_t,set<uint32_t> > &segs_labels_out,    
    map<uint32_t, Supervoxel<PointXYZRGB>::Ptr> &supervoxel_out,
    int num_of_objects    
)
{    
    map<uint32_t,uint32_t> map_l2l;
    map<uint32_t,uint32_t> map_l2l_r;
    for(map<uint32_t, Supervoxel<PointXYZRGB>::Ptr>::iterator
        it_sv = supervoxel_clusters.begin(); 
        it_sv!= supervoxel_clusters.end();    it_sv++ )
    {
        if( map_l2l.find(it_sv->first) == map_l2l.end() )
        {
            uint32_t l_new = map_l2l.size();
            map_l2l.insert(pair<uint32_t,uint32_t>(it_sv->first,l_new));
            map_l2l_r.insert(pair<uint32_t,uint32_t>(l_new,it_sv->first));
        }        
    }

    Mat mat_weights = Mat::zeros( map_l2l.size(), 
                                  map_l2l.size(), CV_32FC1);
    for( map<pair<uint32_t,uint32_t>, float>::iterator
         it_weight  = weights_adjacency.begin(); 
         it_weight != weights_adjacency.end();   it_weight++ )
    {
        uint32_t v1 = map_l2l.find(it_weight->first.first)->second;
        uint32_t v2 = map_l2l.find(it_weight->first.second)->second;

        mat_weights.at<float>(v1,v2) = it_weight->second;
        mat_weights.at<float>(v2,v1) = it_weight->second;
    }
    
    segs_labels_out.clear();
    vector<uint32_t> labels_out;
    SpectralClustering(mat_weights, labels_out, num_of_objects);

    for( size_t l=0; l<labels_out.size(); l++ )
    {
        uint32_t label = map_l2l_r.find(l)->second;
        
        map<uint32_t,set<uint32_t> >::iterator it_seg
         = segs_labels_out.find(labels_out[l]);
        if( it_seg != segs_labels_out.end() )
        {
            it_seg->second.insert(label);
        }
        else
        {
            set<uint32_t> set_labels;
            set_labels.insert(label);
            segs_labels_out.insert(
              pair<uint32_t,set<uint32_t> >(labels_out[l],set_labels));
        }
    }
    
    supervoxel_out.clear();
    for(map<uint32_t, Supervoxel<PointXYZRGB>::Ptr>::iterator
        it_l2sv = supervoxel_clusters.begin(); 
        it_l2sv!= supervoxel_clusters.end();    it_l2sv++ )
    {
        uint32_t label = labels_out[map_l2l.find(it_l2sv->first)->second];
        map<uint32_t, Supervoxel<PointXYZRGB>::Ptr>::iterator it_out
         = supervoxel_out.find(label);
        Supervoxel<PointXYZRGB>::Ptr it_sv;
        if( it_out == supervoxel_out.end() )
        {
            it_sv = Supervoxel<PointXYZRGB>::Ptr(new Supervoxel<PointXYZRGB>);
            supervoxel_out.insert(pair<uint32_t, Supervoxel<PointXYZRGB>::Ptr>
                (supervoxel_out.size(), it_sv) );
        }
        else
        {
            it_sv = it_out->second;
        }

        *it_sv->voxels_ += *it_l2sv->second->voxels_;
    }

    for(map<uint32_t, Supervoxel<PointXYZRGB>::Ptr>::iterator
        it_l2sv = supervoxel_out.begin(); 
        it_l2sv!= supervoxel_out.end();    it_l2sv++ )
    {
        CentroidPoint<PointXYZRGB> cp;
        for(PointCloud<PointXYZRGB>::iterator 
            it_pt = it_l2sv->second->voxels_->begin(); 
            it_pt!= it_l2sv->second->voxels_->end(); it_pt++ )
        {
            cp.add(*it_pt);
        }
        cp.get(it_l2sv->second->centroid_);
    }
}


void calculateAxis(PointCloud<PointXYZRGB>::Ptr pointCloud,
                   PointXYZ &cog,
                   Eigen::Vector3f &principalAxis,
                   Eigen::Vector3f &secondaryAxis,
                   Eigen::Vector3f &trinaryAxis            )
{
    unsigned point_count = static_cast<unsigned> (pointCloud->points.size() );
    cog.x = 0; cog.y = 0; cog.z = 0;
    for( size_t i=0; i<point_count; i++ )
    {
        cog.x += pointCloud->points[i].x;
        cog.y += pointCloud->points[i].y;
        cog.z += pointCloud->points[i].z;
    }
    cog.x = cog.x / point_count;
    cog.y = cog.y / point_count;
    cog.z = cog.z / point_count;

    Eigen::MatrixXf covariance_matrix = Eigen::MatrixXf::Zero(3,3);
    // For each point in the cloud
    for( size_t i=0; i<point_count; i++ )
    {
        pcl::PointXYZ pt;
        pt.x = pointCloud->points[i].x - cog.x;
        pt.y = pointCloud->points[i].y - cog.y;
        pt.z = pointCloud->points[i].z - cog.z;
        covariance_matrix (1, 1) += pt.y * pt.y;
        covariance_matrix (1, 2) += pt.y * pt.z;
        covariance_matrix (2, 2) += pt.z * pt.z;
        covariance_matrix (0, 0) += pt.x * pt.x; 
        covariance_matrix (0, 1) += pt.x * pt.y;
        covariance_matrix (0, 2) += pt.x * pt.z;
    }
    covariance_matrix (1, 0) = covariance_matrix (0, 1);
    covariance_matrix (2, 0) = covariance_matrix (0, 2);
    covariance_matrix (2, 1) = covariance_matrix (1, 2);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> es(3);
    es.compute(covariance_matrix);
    Eigen::MatrixXf axes = es.eigenvectors();
    principalAxis = axes.col(2);
    secondaryAxis = axes.col(1);
    trinaryAxis = axes.col(0);
}

void GetBoundingBox( PointCloud<PointXYZRGB>::Ptr cloud, Mat &camera_K,
                     sensor_msgs::RegionOfInterest &region             )
{
    int min_xx= INFINITY, min_yy= INFINITY, 
        max_xx=-INFINITY, max_yy=-INFINITY;
    for( size_t p=0; p<cloud->size(); p++ )
    {
        PointXYZRGB &pt = (*cloud)[p];
        float x,y;
        Projection_K<PointXYZRGB>(pt,camera_K, &x,&y);
        int r=int(y+0.5), c=int(x+0.5);

        if( min_xx > c ) min_xx = c;
        if( min_yy > r ) min_yy = r;
        if( max_xx < c ) max_xx = c;
        if( max_yy < r ) max_yy = r;
    }

    region.x_offset = min_xx; region.width  = max_xx - min_xx;
    region.y_offset = min_yy; region.height = max_yy - min_yy;
}

void GetSegmentationObjects(
    map<uint32_t, Supervoxel<PointXYZRGB>::Ptr> &map_sv,    
    map<uint32_t,set<uint32_t> > &segs_labels,
    LCCP2DSegParam &param,
    Mat &camera_K, Mat &camera_RT,
    rl_msgs::SegmentationScene &scene
)
{
    // camera parameter
    scene.camera_K.clear();
    for( int r=0; r<camera_K.rows; r++ )
        for( int c=0; c<camera_K.cols; c++ )
            scene.camera_K.push_back(camera_K.at<float>(r,c));

    scene.camera_RT.clear();
    for( int r=0; r<camera_RT.rows; r++ )
        for( int c=0; c<camera_RT.cols; c++ )
            scene.camera_RT.push_back(camera_RT.at<float>(r,c));

    static MeanShift ms;

    set<pair<uint32_t,uint32_t> > edges_label;
    map<uint32_t,uint32_t> seg2obj;
    map<uint32_t,uint32_t> l2i;
    for( map<uint32_t,set<uint32_t> >::iterator
         it_seg = segs_labels.begin(); 
         it_seg != segs_labels.end(); it_seg++
    )
    {
        rl_msgs::SegmentationObject rlso;

        vector<vector<double> > normals;

        PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
        set<uint32_t> &set_labels = it_seg->second;        
        if( set_labels.size() < 2 ) continue;

        geometry_msgs::Point pt_center;
        pt_center.x=0; pt_center.y=0; pt_center.z=0;
        vector<uint32_t> idx2label;

        rlso.voxel_points.clear();
        rlso.voxel_normals.clear();
        for(set<uint32_t>::iterator it_labels = set_labels.begin();
            it_labels != set_labels.end(); it_labels++ )
        {
            uint32_t label = *it_labels;
            idx2label.push_back(label);

            Supervoxel<PointXYZRGB>::Ptr sv = map_sv.find(label)->second;
            *cloud += *(sv->voxels_);

            geometry_msgs::Point pt;
            pt.x = sv->centroid_.x;
            pt.y = sv->centroid_.y;
            pt.z = sv->centroid_.z;
            rlso.voxel_points.push_back(pt);

            pt_center.x += pt.x * sv->voxels_->size();
            pt_center.y += pt.y * sv->voxels_->size();
            pt_center.z += pt.z * sv->voxels_->size();

            geometry_msgs::Vector3 n;
            n.x = sv->normal_.normal_x;
            n.y = sv->normal_.normal_y;
            n.z = sv->normal_.normal_z;
            rlso.voxel_normals.push_back(n);

            vector<double> normal;
            normal.push_back(n.x);
            normal.push_back(n.y);
            normal.push_back(n.z);
            normals.push_back(normal);
        }

        if( cloud->size() < 100 ) continue; // skip small;

        pt_center.x /= cloud->size();
        pt_center.y /= cloud->size();
        pt_center.z /= cloud->size();
        rlso.center = pt_center;
        toROSMsg(*cloud,rlso.cloud);

        if( param.compute_bbox ) GetBoundingBox(cloud,camera_K,rlso.region);        
        if( param.compute_face==false )
        {
            scene.objects.push_back(rlso);
            continue;
        }

        float min_x= INFINITY, min_y= INFINITY, min_z= INFINITY,
              max_x=-INFINITY, max_y=-INFINITY, max_z=-INFINITY;

        map<uint32_t,PointCloud<PointXYZRGB>::Ptr > f2pc; // face -> pointcloud
        vector<Cluster> cs;
        vector<int> idxes;        
        //ms.cluster(normals, 0.3, cs, idxes);
        ms.cluster(normals, 0.4, cs, idxes);
        //ms.cluster(normals, 3, cs, idxes);                
        for( size_t i=0; i<idxes.size(); i++ )
        {
            PointCloud<PointXYZRGB>::Ptr pc;
            if( f2pc.find(idxes[i]) == f2pc.end() )
            {
                pc = PointCloud<PointXYZRGB>::Ptr(new PointCloud<PointXYZRGB>);
                f2pc.insert(
                    pair<uint32_t,PointCloud<PointXYZRGB>::Ptr>(idxes[i], pc));
            }
            else
            {
                pc = f2pc.find(idxes[i])->second;
            }
            *pc += *(map_sv.find(idx2label[i])->second->voxels_);
        }

        rlso.faces.clear();
        for( map<uint32_t,PointCloud<PointXYZRGB>::Ptr>::iterator 
             it_f2pc = f2pc.begin(); it_f2pc != f2pc.end(); it_f2pc++ )
        {
            if( it_f2pc->second->size() < 3 ) continue;

            rl_msgs::SegmentationFace face;
            toROSMsg(*it_f2pc->second,face.cloud);            

            for( size_t i=0; i<idxes.size(); i++ )            
            {
                if( idxes[i] == it_f2pc->first )
                {
                    face.voxel_points.push_back(rlso.voxel_points[i]);
                    face.voxel_normals.push_back(rlso.voxel_normals[i]);
                }
            }

            PointXYZ pt_mean;
            Eigen::Vector3f principalAxis, secondaryAxis, trinaryAxis; 
            calculateAxis(it_f2pc->second, 
                          pt_mean,principalAxis,secondaryAxis,trinaryAxis);

            float max_width_principal = -INFINITY;
            float min_width_principal =  INFINITY;
            float max_width_secondary = -INFINITY;
            float min_width_secondary =  INFINITY;
            float max_width_trinary   = -INFINITY;
            float min_width_trinary   =  INFINITY;
            for( size_t p=0; p<cloud->size(); p++ )
            {
                PointXYZRGB &pt = (*cloud)[p];

                float width_principal 
                  = principalAxis[0] * (pt.x - pt_mean.x) +
                    principalAxis[1] * (pt.y - pt_mean.y) +
                    principalAxis[2] * (pt.z - pt_mean.z);

                float width_secondary
                  = secondaryAxis[0] * (pt.x - pt_mean.x) +
                    secondaryAxis[1] * (pt.y - pt_mean.y) +
                    secondaryAxis[2] * (pt.z - pt_mean.z);

                float width_trinary
                  = trinaryAxis[0]   * (pt.x - pt_mean.x) +
                    trinaryAxis[1]   * (pt.y - pt_mean.y) +
                    trinaryAxis[2]   * (pt.z - pt_mean.z);

                if( max_width_principal < width_principal )
                    max_width_principal = width_principal;
                if( min_width_principal > width_principal )
                    min_width_principal = width_principal;
                if( max_width_secondary < width_secondary )
                    max_width_secondary = width_secondary;
                if( min_width_secondary > width_secondary )
                    min_width_secondary = width_secondary;
                if( max_width_trinary   < width_trinary   )
                    max_width_trinary   = width_trinary;
                if( min_width_trinary   > width_trinary   )
                    min_width_trinary   = width_trinary;

                // global bounding box
                if( min_x > (pt.x) ) min_x=(pt.x);
                if( min_y > (pt.y) ) min_y=(pt.y);
                if( min_z > (pt.z) ) min_z=(pt.z);
                if( max_x < (pt.x) ) max_x=(pt.x);
                if( max_y < (pt.y) ) max_y=(pt.y);
                if( max_z < (pt.z) ) max_z=(pt.z);
            }

            PointXYZ pt_11( pt_mean.x + principalAxis[0] * max_width_principal,
                            pt_mean.y + principalAxis[1] * max_width_principal,
                            pt_mean.z + principalAxis[2] * max_width_principal);
            PointXYZ pt_12( pt_mean.x + principalAxis[0] * min_width_principal,
                            pt_mean.y + principalAxis[1] * min_width_principal,
                            pt_mean.z + principalAxis[2] * min_width_principal);

            PointXYZ pt_21( pt_mean.x + secondaryAxis[0] * max_width_secondary,
                            pt_mean.y + secondaryAxis[1] * max_width_secondary,
                            pt_mean.z + secondaryAxis[2] * max_width_secondary);
            PointXYZ pt_22( pt_mean.x + secondaryAxis[0] * min_width_secondary,
                            pt_mean.y + secondaryAxis[1] * min_width_secondary,
                            pt_mean.z + secondaryAxis[2] * min_width_secondary);

            PointXYZ pt_31( pt_mean.x + trinaryAxis[0] * max_width_trinary,
                            pt_mean.y + trinaryAxis[1] * max_width_trinary,
                            pt_mean.z + trinaryAxis[2] * max_width_trinary);
            PointXYZ pt_32( pt_mean.x + trinaryAxis[0] * min_width_trinary,
                            pt_mean.y + trinaryAxis[1] * min_width_trinary,
                            pt_mean.z + trinaryAxis[2] * min_width_trinary);

            float dist_1 = sqrt( (pt_11.x-pt_12.x)*(pt_11.x-pt_12.x) +
                                 (pt_11.y-pt_12.y)*(pt_11.y-pt_12.y) +
                                 (pt_11.z-pt_12.z)*(pt_11.z-pt_12.z)   )/2.0;

            float dist_2 = sqrt( (pt_21.x-pt_22.x)*(pt_21.x-pt_22.x) +
                                 (pt_21.y-pt_22.y)*(pt_21.y-pt_22.y) +
                                 (pt_21.z-pt_22.z)*(pt_21.z-pt_22.z)   )/2.0;

            float dist_3 = sqrt( (pt_31.x-pt_32.x)*(pt_31.x-pt_32.x) +
                                 (pt_31.y-pt_32.y)*(pt_31.y-pt_32.y) +
                                 (pt_31.z-pt_32.z)*(pt_31.z-pt_32.z)   )/2.0;

            face.center.x = (pt_11.x+pt_12.x + pt_21.x+pt_22.x)/2.0 - pt_mean.x;
            face.center.y = (pt_11.y+pt_12.y + pt_21.y+pt_22.y)/2.0 - pt_mean.y;
            face.center.z = (pt_11.z+pt_12.z + pt_21.z+pt_22.z)/2.0 - pt_mean.z;

            face.normal.x = cs[it_f2pc->first].mode[0];
            face.normal.y = cs[it_f2pc->first].mode[1];
            face.normal.z = cs[it_f2pc->first].mode[2];
            double norm = sqrt( (face.normal.x * face.normal.x) + 
                                (face.normal.y * face.normal.y) + 
                                (face.normal.z * face.normal.z)   );
            face.normal.x /= norm;
            face.normal.y /= norm;
            face.normal.z /= norm;

            face.vecs_axis.clear();
            face.sizes.clear();

            geometry_msgs::Vector3 vec1;
            vec1.x = principalAxis[0];
            vec1.y = principalAxis[1];
            vec1.z = principalAxis[2];
            face.vecs_axis.push_back(vec1);
            face.sizes.push_back(dist_1);
            
            geometry_msgs::Vector3 vec2;
            vec2.x = secondaryAxis[0];
            vec2.y = secondaryAxis[1];
            vec2.z = secondaryAxis[2];
            face.vecs_axis.push_back(vec2);
            face.sizes.push_back(dist_2);
            
            geometry_msgs::Vector3 vec3;
            vec3.x = trinaryAxis[0];
            vec3.y = trinaryAxis[1];
            vec3.z = trinaryAxis[2];
            face.vecs_axis.push_back(vec3);
            face.sizes.push_back(dist_3);
            
            rlso.faces.push_back(face);
        }

        // skip small
        bool skip=true;
        for( size_t f=0; f<rlso.faces.size(); f++ )
        {
            if( rlso.faces[f].sizes[0] * rlso.faces[f].sizes[1] > 0.0004 )
            {
                skip = false;
                break;
            }
        }
        if( skip ) continue;


        seg2obj.insert(pair<uint32_t,uint32_t>(it_seg->first,scene.objects.size()));
        scene.objects.push_back(rlso);
    }
}

SegLCCP2DSeg::SegLCCP2DSeg() : idx_save(0)
{

}

SegLCCP2DSeg::SegLCCP2DSeg(ros::NodeHandle* nh, std::string SUB_COB)
 : idx_save{0}
{
    clt_cob = nh->serviceClient<rl_msgs::seg_cob_srv>(SUB_COB);
};

static void copyTo(const vector<float> &vec, Mat &mat)
{
    assert(mat.cols*mat.rows == vec.size());
    memcpy(mat.data, vec.data(), vec.size()*sizeof(float));    
}

bool SegLCCP2DSeg::Segment( 
    LCCP2DSegParam               &param,
    SUPERVOXELS                  &supervoxel_clusters,
    WEIGHTED_EDGES               &weights_adjacency,
    map<uint32_t,set<uint32_t> > &segs_labels,
    Mat                          &image_seg,
    Mat                          &image_sv,
    vector<shape_msgs::Plane>    &planes_bg,
    const Mat &image, 
    const PointCloud<PointXYZRGB> &cloud_input_,
    const string &param_str,
    const vector<float> &camera_K_,
    const vector<float> &camera_RT_
)
{
    //LCCP2DSegParam param;
    param.parse_param(param_str);
    stringstream ss;
    param.print(ss);
    ROS_INFO_STREAM(endl << ss.str());

    // Intrinsic Camera Parameter
    Mat camera_K(3,3,CV_32F);
    copyTo(camera_K_,  camera_K);    

    PointCloud<PointXYZRGB>::Ptr cloud_input(new PointCloud<PointXYZRGB>);
    copyPointCloud(cloud_input_, *cloud_input);
    
    // Remove Invalid points    
    PassThrough<PointXYZRGB> pass;
    pass.setInputCloud (cloud_input);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.1, 2);
    pass.filter (*cloud_input);    
    pcl::StatisticalOutlierRemoval<PointXYZRGB> sor_noise;
    sor_noise.setInputCloud (cloud_input);
    sor_noise.setMeanK (70);
    sor_noise.setStddevMulThresh (1);
    sor_noise.filter (*cloud_input);

    // Downsampling for speed
#if 0
    if( param.downsampling )
    {
        ROS_INFO_STREAM("Downsampling from " << cloud_input->size());
        VoxelGrid< PointXYZRGB > sor;
        sor.setInputCloud (cloud_input);
        sor.setLeafSize (0.0025f, 0.0025f, 0.0025f);
        sor.filter (*cloud_input);
        ROS_INFO_STREAM("Downsampling to " << cloud_input->size() << " - Done");        
    }
#else
    {
        ROS_INFO_STREAM("Downsampling from " << cloud_input->size());
        utils::PointCloud2PointCloudVox(*cloud_input,*cloud_input, 0.0025);
        ROS_INFO_STREAM("Downsampling to " << cloud_input->size() << " - Done");
    }
#endif

    // Remove Background    
    if( param.remove_background )
    {
        ROS_INFO_STREAM("Remove Background/Noise");        
        RemoveBackground(cloud_input, planes_bg, camera_RT_);
        ROS_INFO_STREAM("Remove Background/Noise - Done");
    }    
    if( cloud_input->size() <= 0 )
    {
        ROS_WARN_STREAM("Too small # of point clouds");        
        return false;
    }

    // Remove point ourside of the workspace
    if( camera_RT_.size() == 16 && param.workspace.size() == 10 )
    {
        ROS_INFO_STREAM("Crop workspace " << cloud_input->size());
        RemoveBackground(cloud_input, param.workspace, camera_RT_);
        ROS_INFO_STREAM("Crop workspace " << cloud_input->size() << " - Done");
    }

    // Create Supervoxel
    //map<uint32_t, Supervoxel<PointXYZRGB>::Ptr> supervoxel_clusters;
    map<uint32_t, float>                        supervoxel_confidences;
    multimap<uint32_t, uint32_t>                supervoxel_adjacency;
    //map<pair<uint32_t,uint32_t>, float>            weights_adjacency;    

    if( param.use_supervoxel ) // Create Supervoxel - using the traditional way
    {
        SuperVoxelParam param_sv;        
        param_sv.parse_param(param_str);

        ROS_INFO("Create Supervoxels");
        DoSupervoxelClustering(cloud_input, param_sv, 
                               &supervoxel_clusters, 
                               &supervoxel_adjacency );

        for( SUPERVOXELS::iterator it_l2sv = supervoxel_clusters.begin();
             it_l2sv != supervoxel_clusters.end(); it_l2sv++ )
        {
            supervoxel_confidences.insert(pair<uint32_t,float>(it_l2sv->first,1));            
        }
        ROS_INFO("Create Supervoxels - Done");
    }
    else // Create Supervoxel - using 2D segmentation        
    {        
        // Get 2D Image Segmentation
        ROS_INFO_STREAM("Get 2D Segmentation [" << param.name_2dseg << "]");        
        if( param.name_2dseg.compare("cob")==0 )
        {
            sensor_msgs::Image msg_img;
            cv_bridge::CvImage cv_image;    
            cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            cv_image.image    = image;
            cv_image.toImageMsg(msg_img);

            rl_msgs::seg_cob_srv srv_cob;
            srv_cob.request.image = msg_img;
            if( clt_cob.call(srv_cob) )
            {
                cv_bridge::CvImagePtr cv_ptr
                 = cv_bridge::toCvCopy(srv_cob.response.images_seg[1],
                                       sensor_msgs::image_encodings::MONO16 );
                cv_ptr->image.copyTo(image_sv);
            }
            else
            {
                ROS_ERROR_STREAM("Failed get segmentation [cob]");
                return false;
            }
        }
        else if( param.name_2dseg.compare("quickshift")==0 )
        {
            Segmentation_QuickShift(image, image_sv);        
        }
        else
        {
            ROS_ERROR_STREAM("Unsupported Segmentation Method");
        }
        ROS_INFO_STREAM("Get 2D Segmentation - Done");

        ROS_INFO_STREAM("Create Voxels from 2D Seg");        
        DoSupervoxel_from_2Dseg(cloud_input,
                                image_sv, camera_K,                                 
                                supervoxel_clusters, 
                                supervoxel_confidences,
                                supervoxel_adjacency);
        ROS_INFO_STREAM("Create Voxels from 2D Seg - Done");
    }

    ROS_INFO("Measure the Convexity");
    ComputeConvexities( supervoxel_clusters,
                        supervoxel_confidences,
                        supervoxel_adjacency, 
                        weights_adjacency, 1  ); // 1hop convexity
    ROS_INFO("Measure the Convexity - Done");

/*
    ROS_INFO("Weight the Strongly Connected Edges");
    WeightStronglyConnectedEdges( images_seg[1], 
                                  supervoxel_clusters,
                                  supervoxel_adjacency,    
                                  camera_K,
                                  weights_adjacency     );
    ROS_INFO("Weight the Strongly Connected Edges - Done");
*/

/*
    ROS_INFO("Find the Strongly Connected Edges");
    multimap<uint32_t,uint32_t> edges_strong;
    GetStronglyConnectedEdges(images_seg[1], edges_strong);
    ROS_INFO("Find the Strongly Connected Edges - Done");
*/

    ROS_INFO("Spectral Clustering");
    map<uint32_t, Supervoxel<PointXYZRGB>::Ptr> supervoxels_out;    
    //map<uint32_t,set<uint32_t> > segs_labels;
    SpectralClustering( supervoxel_clusters, weights_adjacency, segs_labels, 
                        supervoxels_out, param.num_of_objects);
    ROS_INFO("Spectral Clustering - Done");

#if 1
    ROS_INFO("Post-Processing for Small Segments");
    {        
        vector<map<uint32_t,set<uint32_t> >::iterator> tobeDelete;
        for(map<uint32_t,set<uint32_t> >::iterator it_seg = segs_labels.begin(); 
            it_seg != segs_labels.end(); it_seg++ )
        {
            uint32_t label_seg = it_seg->first;
            set<uint32_t> &set_labels = it_seg->second;

            PointCloud<PointXYZRGB> cloud_tmp;
            for(set<uint32_t>::iterator it_labels = set_labels.begin();
                it_labels != set_labels.end(); it_labels++ )
            {
                uint32_t label_sv = *it_labels;
                SUPERVOXELS::iterator it_sv = supervoxel_clusters.find(label_sv);

                PointXYZRGB pt1, pt2;
                getMinMax3D (*it_sv->second->voxels_, pt1, pt2);
                cloud_tmp.push_back(pt1);
                cloud_tmp.push_back(pt2);
            }

            // Measure the approximated size
            PointXYZRGB minPt, maxPt;
            getMinMax3D (cloud_tmp, minPt, maxPt);
            float dist = sqrt( (maxPt.x-minPt.x)*(maxPt.x-minPt.x)+
                               (maxPt.y-minPt.y)*(maxPt.y-minPt.y)+
                               (maxPt.z-minPt.z)*(maxPt.z-minPt.z)  );
            if( dist > 0.05 ) continue;

            // Merge to bigger one

            map<set<uint32_t>*,int> map_set_count;

            // check all the adjacent supervoxels            
            for( set<uint32_t>::iterator it_labels = set_labels.begin();
                 it_labels != set_labels.end(); it_labels++ )
            {
                pair< multimap<uint32_t,uint32_t>::iterator,
                      multimap<uint32_t,uint32_t>::iterator  > ret
                 = supervoxel_adjacency.equal_range(*it_labels);
                for( multimap<uint32_t,uint32_t>::iterator it_adj=ret.first;
                     it_adj!=ret.second; it_adj++ )
                {
                    set<uint32_t>::iterator it_set
                     = set_labels.find(it_adj->second);
                    if( it_set != set_labels.end() ) continue;
                    
                    for( map<uint32_t,set<uint32_t> >::iterator it_seg2
                          = segs_labels.begin(); 
                         it_seg2 != segs_labels.end(); it_seg2++ )
                    {                            
                        if( it_seg2->second.find(it_adj->second)
                             == it_seg2->second.end() ) continue;

                        map<set<uint32_t>*,int>::iterator it_cnt
                         = map_set_count.find(&it_seg2->second);
                        if( it_cnt == map_set_count.end() )
                        {
                            map_set_count.insert(
                              pair<set<uint32_t>*,int>(&it_seg2->second,1));
                        }
                        else
                        {
                            it_cnt->second++;
                        }
                    }
                }
            }

            map<set<uint32_t>*,int>::iterator it_max = map_set_count.end();
            int cnt_max = -1;
            for( map<set<uint32_t>*,int>::iterator it_cnt
                 = map_set_count.begin(); it_cnt!=map_set_count.end(); it_cnt++)
            {
                if( cnt_max < it_cnt->second )
                {
                    cnt_max = it_cnt->second;
                    it_max = it_cnt;
                }
            }

            if( it_max != map_set_count.end() )
            {
                tobeDelete.push_back(it_seg);
                for( set<uint32_t>::iterator it_labels = set_labels.begin();
                     it_labels != set_labels.end(); it_labels++ )
                {
                    it_max->first->insert(*it_labels);
                }
            }
        }

        for( int d=0; d<tobeDelete.size(); d++ )
        {
            segs_labels.erase(tobeDelete[d]);
        }
    }
    ROS_INFO("Post-Processing for Small Segments - Done");
#endif 
    return true;
}

void DepthFirstLabeling(const Mat &image_sv, int r, int c, ushort label, 
                        Mat &image_seg)
{
    if( image_seg.at<ushort>(r,c)==label ) return;

    cout << "r:" << r << ", c:"  << c << endl;
    image_seg.at<ushort>(r,c) = label;

    ushort l  = image_sv.at<ushort>(r,c);

    
    int r1=r-1, c1=c;
    if( 0<=r1 && 0<=c1 && r1<image_sv.rows && c1<image_sv.cols )
    {
        cout << "label: " << label << ", label2: " << image_sv.at<ushort>(r1,c1) << endl;
        ushort l1 = image_sv.at<ushort>(r1,c1);
        if( l==l1 )
        {            
            DepthFirstLabeling(image_sv, r1,c1,label, image_seg );
        }
    }

    int r2=r+1, c2=c;
    if( 0<=r2 && 0<=c2 && r2<image_sv.rows && c2<image_sv.cols )
    {
        ushort l2 = image_sv.at<ushort>(r2,c2);
        if( l==l2 )
        {            
            DepthFirstLabeling(image_sv, r2,c2,label, image_seg );
        }
    }

    int r3=r,   c3=c-1;
    if( 0<=r3 && 0<=c3 && r3<image_sv.rows && c3<image_sv.cols )
    {
        ushort l3 = image_sv.at<ushort>(r3,c3);
        if( l==l3 )
        {
            DepthFirstLabeling(image_sv, r3,c3,label, image_seg );
        }
    }

    int r4=r,   c4=c+1;
    if( 0<=r4 && 0<=c4 && r4<image_sv.rows && c4<image_sv.cols )
    {
        ushort l4 = image_sv.at<ushort>(r4,c4);
        if( l==l4 )
        {            
            DepthFirstLabeling(image_sv, r4,c4,label, image_seg );
        }
    }
}

bool SegLCCP2DSeg::Segment(Mat &image_seg,
                           const Mat &image,
                           const Mat &depth,
                           const PointCloud<PointXYZRGB> &cloud_input, 
                           const string &param_str,
                           const vector<float> &camera_K_,
                           const vector<float> &camera_RT_, 
                           const float depth_scale                     )
{
    Mat camera_K(3,3,CV_32F), camera_RT(4,4,CV_32F);

    if( camera_K_.size()==0 ) camera_K = Mat::eye(3,3, CV_32F);
    else                      copyTo(camera_K_,  camera_K);
    if( camera_RT_.size()==0 ) camera_RT = Mat::eye(4,4, CV_32F);
    else                       copyTo(camera_RT_, camera_RT);

    LCCP2DSegParam param;
    SUPERVOXELS                  supervoxel_clusters;
    WEIGHTED_EDGES               weights_adjacency;
    map<uint32_t,set<uint32_t> > segs_labels;
    vector<shape_msgs::Plane>    planes_bg;
    Mat                          image_sv;

    Segment(param, supervoxel_clusters, weights_adjacency, segs_labels, 
            image_seg, image_sv, planes_bg,
            image, cloud_input, param_str, camera_K_, camera_RT_ );

    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);    
    for( map<uint32_t,set<uint32_t> >::iterator
         it_seg = segs_labels.begin(); 
         it_seg != segs_labels.end(); it_seg++
    )
    {
        uint32_t label_seg = it_seg->first;
        set<uint32_t> &set_labels = it_seg->second;
        
        if( set_labels.size() < 2 ) continue;

        for(set<uint32_t>::iterator it_labels = set_labels.begin();
            it_labels != set_labels.end(); it_labels++ )
        {
            uint32_t label_sv = *it_labels;

            map<uint32_t, Supervoxel<PointXYZRGB>::Ptr>::iterator
            it_sv = supervoxel_clusters.find(label_sv);
            PointXYZRGBA &pt_ctr = it_sv->second->centroid_;

            PointCloud<PointXYZRGB>::Ptr voxels = it_sv->second->voxels_;
            for(size_t p=0; p<voxels->size(); p++)
            {
                PointXYZRGB pt_tmp;
                pt_tmp.x = voxels->points[p].x; 
                pt_tmp.y = voxels->points[p].y;
                pt_tmp.z = voxels->points[p].z;
                pt_tmp.r = label_seg+1;
                cloud->push_back(pt_tmp);
            }
        }
    }

    vector<int> idx(1);
    vector<float> dist(1);
    KdTreeFLANN<PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloud);
    image_seg = Mat::zeros(depth.rows, depth.cols, CV_16U);
    for( int c=0; c<depth.cols; c++ )
    for( int r=0; r<depth.rows; r++ )
    {
        //float z = (float)depth.at<ushort>(r,c) * 1.25 * 0.0001;
        float z = (float)depth.at<ushort>(r,c) * depth_scale;
        if( z<0.01 ) continue;

        float x = ((float)c-camera_K.at<float>(0,2))/camera_K.at<float>(0,0)*z;
        float y = ((float)r-camera_K.at<float>(1,2))/camera_K.at<float>(1,1)*z;
        PointXYZRGB pt;
        pt.x=x; pt.y=y; pt.z=z;

        kdtree.nearestKSearch(pt, 1, idx, dist);
        if( dist[0] < 0.0001 )
        {
            image_seg.at<ushort>(r,c) = (ushort)cloud->points[idx[0]].r;
        }
    }
}

bool SegLCCP2DSeg::Segment(rl_msgs::SegmentationScene* segscene_out,
                           sensor_msgs::Image &msg_img,
                           sensor_msgs::Image &msg_dep,
                           const float depth_scale,                           
                           const vector<float> &camera_K,
                           const vector<float> &camera_RT,
                           const string &param_str          )
{
    PointCloud<PointXYZRGB>::Ptr cloud_cam(new PointCloud<PointXYZRGB>);

    cv_bridge::CvImagePtr cv_img
     = cv_bridge::toCvCopy(msg_img, msg_img.encoding);    

    cv_bridge::CvImagePtr cv_dep
     = cv_bridge::toCvCopy(msg_dep, msg_dep.encoding);

    int bitdepth = sensor_msgs::image_encodings::bitDepth(msg_dep.encoding);
    if( bitdepth==16 )
    {
        utils::PointCloudfromDepth<PointXYZRGB, uint16_t>(
            *cloud_cam, cv_dep->image, depth_scale,
            camera_K, vector<float>(), cv_img->image, cv::Mat() );
    }    
    else
    {
        ROS_ERROR_STREAM("Unknown Depth Encoding: " << msg_dep.encoding);
        return false;
    }

/*
    cout << "camera_K: ";
    for( int i=0; i<9; i++ ) cout << camera_K[i] << " ";
    cout << endl;

    cout << "camera_RT.size(): " << camera_RT.size() << endl;
    if( camera_RT.size() )
    {
        for( int i=0; i<16; i++ ) cout << camera_RT[i] << " ";
    }
    cout << "depth_scale: " << depth_scale << endl;
    cout << "cloud.size(): " << cloud_cam->size() << endl;
    cout << "param:  " << param_str << endl;
*/

    return Segment(segscene_out, msg_img, cloud_cam, param_str, camera_K,camera_RT);
}

bool SegLCCP2DSeg::Segment(rl_msgs::SegmentationScene* segscene_out,
                           sensor_msgs::Image &msg_img,
                           PointCloud<PointXYZRGB>::Ptr cloud_input, 
                           const string &param_str,
                           const vector<float> &camera_K_,
                           const vector<float> &camera_RT_          )
{
    Mat camera_K(3,3,CV_32F), camera_RT(4,4,CV_32F);

    if( camera_K_.size()==0 ) camera_K = Mat::eye(3,3, CV_32F);
    else                      copyTo(camera_K_,  camera_K);
    if( camera_RT_.size()==0 ) camera_RT = Mat::eye(4,4, CV_32F);
    else                       copyTo(camera_RT_, camera_RT);

    LCCP2DSegParam param;
    SUPERVOXELS                  supervoxel_clusters;
    WEIGHTED_EDGES               weights_adjacency;
    map<uint32_t,set<uint32_t> > segs_labels;
    Mat                          image_seg, image_seg2d;

    cv_bridge::CvImagePtr cv_ptr
     = cv_bridge::toCvCopy(msg_img,
                           sensor_msgs::image_encodings::BGR8 );
    Segment(param, supervoxel_clusters, weights_adjacency, segs_labels, 
            image_seg, image_seg2d, segscene_out->planes_bg,
            cv_ptr->image, *cloud_input, param_str, camera_K_, camera_RT_ );
     
    //segscene_out->objects.clear();

    ROS_INFO("Convert to ROS msg");
    //GetSegmentationObjects(supervoxel_clusters,supervoxel_adjacency,segs_labels, 
    GetSegmentationObjects(supervoxel_clusters,segs_labels, 
                           param, camera_K, camera_RT,
                           *segscene_out);
    ROS_INFO("Convert to ROS msg - Done");    

    if( param.save_snapshot )
    {
        idx_save++;
        char fp_rgb_save[256];
        sprintf(fp_rgb_save, param.strfmt_save.c_str(), 
                idx_save, "color", "png");
        cv_bridge::CvImagePtr cv_ptr        
         = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8);
        imwrite(fp_rgb_save, cv_ptr->image);
        ROS_INFO_STREAM("saved: " << fp_rgb_save);

        char fp_input_ply_save[256];
        sprintf(fp_input_ply_save, param.strfmt_save.c_str(), 
                idx_save, "input", "ply");
        io::savePLYFileBinary(fp_input_ply_save, *cloud_input);

        char fp_input_pcd_save[256];
        sprintf(fp_input_pcd_save, param.strfmt_save.c_str(), 
                idx_save, "input", "pcd");
        io::savePCDFileBinary(fp_input_pcd_save, *cloud_input);

        if( !param.use_supervoxel )
        {
            char fp_seg2d_save[256];
            sprintf(fp_seg2d_save, param.strfmt_save.c_str(),
                    idx_save, param.name_2dseg.c_str(), "png");
            Mat img;
            drawLabels(image_seg,img);
            imwrite(fp_seg2d_save, img);
            ROS_INFO_STREAM("saved: " << fp_seg2d_save);
        }

        char fp_cloud_save[256];
        sprintf(fp_cloud_save, param.strfmt_save.c_str(), 
                idx_save, "cloud", "png");
        visualization::PCLVisualizer viewer_cloud("Cloud");
        //viewer_cloud.setSize(1200,960);
        viewer_cloud.setSize(600,480);        
        viewer_cloud.setCameraPosition(0,0,-0.5,0,0,1,0,-1,0);
        viewer_cloud.setBackgroundColor(1,1,1);
        viewer_cloud.addPointCloud(cloud_input,"cloud");
        viewer_cloud.setPointCloudRenderingProperties(
                 visualization::PCL_VISUALIZER_POINT_SIZE,3,"cloud");
        viewer_cloud.spinOnce(100);
        viewer_cloud.saveScreenshot(fp_cloud_save);        
        viewer_cloud.close();

        char fp_seg_save[256];
        sprintf(fp_seg_save, param.strfmt_save.c_str(), 
                idx_save, "seg", "png");

        visualization::PCLVisualizer viewer_seg("Seg");
        viewer_seg.setSize(600,480);
        //addSupervoxelToViewer(supervoxels_out, viewer_seg);
        addSupervoxelToViewer(supervoxel_clusters, viewer_seg);
        viewer_seg.setCameraPosition(0,0,-0.5,0,0,1,0,-1,0);
        viewer_seg.setBackgroundColor(1,1,1);
        viewer_seg.spinOnce(100);
        viewer_seg.saveScreenshot(fp_seg_save);        
        viewer_seg.close();

        char fp_seg2_save[256];
        sprintf(fp_seg2_save, param.strfmt_save.c_str(), 
                idx_save, "seg2", "png");

        visualization::PCLVisualizer viewer_seg2("Seg2");        
        AddSegmentationObjects(segscene_out->objects,viewer_seg2);
        viewer_seg2.setCameraPosition(0,0,-0.5,0,0,1,0,-1,0);
        viewer_seg2.setBackgroundColor(1,1,1);        
        viewer_seg2.spinOnce(100);
        viewer_seg2.setSize(600,480);
        viewer_seg2.saveScreenshot(fp_seg2_save);        
        viewer_seg2.close();        

        char fp_supervoxel_save[256];
        sprintf(fp_supervoxel_save, param.strfmt_save.c_str(), 
                idx_save, "supervoxel", "png");

        visualization::PCLVisualizer viewer_sv("Supervoxel");
        //viewer_sv.setSize(1200,960);        
        viewer_sv.setSize(600,480);
        addSupervoxelToViewer(supervoxel_clusters, viewer_sv);
        //addSupervoxelIndexToViewer(supervoxel_clusters, viewer_sv);
        //addSupervoxelGraphToViewer(supervoxel_clusters, 
        //                           supervoxel_adjacency, 
        //                           weights_adjacency,    viewer_sv);
        viewer_sv.setCameraPosition(0,0,-0.5,0,0,1,0,-1,0);
        viewer_sv.setBackgroundColor(1,1,1);
        viewer_sv.spinOnce(100);        
        viewer_sv.saveScreenshot(fp_supervoxel_save);
        viewer_sv.close();

        char fp_graph_save[256];
        sprintf(fp_graph_save, param.strfmt_save.c_str(), 
                idx_save, "graph", "png");

        visualization::PCLVisualizer viewer("Graph");
        viewer.setSize(600,480);
        //addSupervoxelToViewer(supervoxel_clusters, viewer);
        //addSupervoxelIndexToViewer(supervoxel_clusters, viewer);
        addSupervoxelGraphToViewer(supervoxel_clusters,
                                   weights_adjacency,    viewer);
        viewer.setCameraPosition(0,0,-0.5,0,0,1,0,-1,0);
        viewer.setBackgroundColor(1,1,1);
        viewer.spinOnce(100);        
        viewer.saveScreenshot(fp_graph_save);
        viewer.close();
    }

    return true;
}