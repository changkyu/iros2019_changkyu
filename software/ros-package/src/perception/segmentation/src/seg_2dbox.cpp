#include <boost/program_options.hpp>

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>

#include "utils/utils_inline.hpp"
#include "utils/utils.hpp"
#include "utils/utils_visualization.hpp"

#include "segmentation/seg_2dbox.hpp"

using namespace std;
using namespace pcl;
//using namespace changkyu;

Seg2DBox::Seg2DBox()
{

}

Seg2DBox::~Seg2DBox()
{

}

static Eigen::Vector3f vec_z(0,0,1);

template<typename PointT>
bool Seg2DBox::Split_helper( 
    vector<pair<float,vector<typename pcl::PointCloud<PointT>::Ptr> > > &clouds_res,                
    float error,
    const typename pcl::PointCloud<PointT>::Ptr cloud_face,
    const Eigen::Vector3f &vec_normal,
    const boxinfo_t &boxinfo,
    const float resolution                        )
{
    bool succ_all = false;

    if( cloud_face->size() == 0 )
    {        
        pair<float,vector<PointCloud<PointXYZRGB>::Ptr> > res;
        res.first = error;        
        clouds_res.push_back(res);

        //cout << "Success!" << endl;
        return true;
    } 

    const float resolution_sq = resolution * resolution;
    const float area_thresh = boxinfo.area * 0.8;
    int thresh = max(boxinfo.width, boxinfo.height) / resolution * 2;

    // project face to plane
    ModelCoefficients::Ptr coef(new ModelCoefficients);            
    GetPlane(vec_normal[0],vec_normal[1],vec_normal[2],cloud_face->points[0],*coef);

    PointCloud<PointXYZRGB>::Ptr cloud_prj(new PointCloud<PointXYZRGB>);
    ProjectInliers<PointXYZRGB> proj_prj;
    proj_prj.setModelType(SACMODEL_PLANE);
    proj_prj.setModelCoefficients(coef);
    proj_prj.setInputCloud(cloud_face);
    proj_prj.filter(*cloud_prj);

    // project to xy plane for accuracy
    Eigen::Quaternionf q = Eigen::Quaternionf::FromTwoVectors(vec_normal,vec_z);
    q.normalize();
    Eigen::Matrix3f tf_zrj3x3 = q.toRotationMatrix();
    Eigen::Matrix4f tf_zrj4x4;
    tf_zrj4x4 <<  tf_zrj3x3(0,0), tf_zrj3x3(0,1), tf_zrj3x3(0,2), 0,
                  tf_zrj3x3(1,0), tf_zrj3x3(1,1), tf_zrj3x3(1,2), 0,
                  tf_zrj3x3(2,0), tf_zrj3x3(2,1), tf_zrj3x3(2,2), 0,
                               0,              0,              0, 1;
    Eigen::Matrix4f tf_zrj4x4_inv = tf_zrj4x4.inverse();
    transformPointCloud(*cloud_prj, *cloud_prj, tf_zrj4x4);

    PointCloud<PointXYZRGB>::Ptr cloudvox_prj(new PointCloud<PointXYZRGB>);
    utils::PointCloud2PointCloudVox(*cloud_prj, *cloudvox_prj, resolution);

    float radius = max(boxinfo.width, boxinfo.height) * 0.5;
    RadiusOutlierRemoval<PointXYZRGB> outrem;
    outrem.setInputCloud(cloudvox_prj);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius( radius*radius / resolution_sq * 0.75 ); // pi / 4
    outrem.filter (*cloudvox_prj);

    if( cloudvox_prj->size() < thresh )
    {
        float err = abs(cloudvox_prj->size()*resolution_sq - boxinfo.area);

        pair<float,vector<PointCloud<PointXYZRGB>::Ptr> > res;
        res.first = err + error;
        res.second.push_back(cloud_face);
        clouds_res.push_back(res);

        //cout << "Success!!" << endl;
        return true;
    }

    // Pick a corner
    PointXYZRGB pt_center_prj;
    pt_center_prj.x = 0;
    pt_center_prj.y = 0;
    pt_center_prj.z = 0;
    for( int p=0; p<cloudvox_prj->size(); p++ )
    {
        pt_center_prj.x += cloudvox_prj->points[p].x;
        pt_center_prj.y += cloudvox_prj->points[p].y;
        pt_center_prj.z += cloudvox_prj->points[p].z;
    }
    pt_center_prj.x /= cloudvox_prj->size();
    pt_center_prj.y /= cloudvox_prj->size();
    pt_center_prj.z /= cloudvox_prj->size();

    float dist_max = -1;    
    vector<PointXYZRGB> pts_corner(3);
    vector<int> p_corners(3);
    for( int p=0; p<cloudvox_prj->size(); p++ )
    {
        PointXYZRGB &pt = cloudvox_prj->points[p];
        float dist = compute_dist(pt_center_prj,pt);
        if( dist_max < dist )
        {
            p_corners[0] = p;
            dist_max = dist;
        }
    }
    pts_corner[0] = cloudvox_prj->points[p_corners[0]];

    dist_max = -1;
    for( int p=0; p<cloudvox_prj->size(); p++ )
    {
        PointXYZRGB &pt = cloudvox_prj->points[p];
        float dist = compute_dist(pts_corner[0],pt);
        if( dist_max < dist )
        {
            p_corners[1] = p;
            dist_max = dist;
        }
    }
    pts_corner[1] = cloudvox_prj->points[p_corners[1]];

    dist_max = -1;
    for( int p=0; p<cloudvox_prj->size(); p++ )
    {
        PointXYZRGB &pt = cloudvox_prj->points[p];
        float dist = compute_dist_Point2LineSegment(pt, pts_corner[0], pts_corner[1]);
        if( dist_max < dist )
        {
            p_corners[2] = p;
            dist_max = dist;
        }
    }
    pts_corner[2] = cloudvox_prj->points[p_corners[2]];

    for( int c=0; c<p_corners.size(); c++ )
    {
        PointXYZRGB pt_corner = cloudvox_prj->points[p_corners[c]];

        // crop points around the corner
        float dist_crop = min(boxinfo.width, boxinfo.height);
        PointCloud<PointXYZRGB>::Ptr cloud_corner(new PointCloud<PointXYZRGB>);
        for( int p=0; p<cloud_prj->size(); p++ )
        {
            PointXYZRGB &pt = cloud_prj->points[p];

            float dist = compute_dist(pt, pt_corner);
            if( dist < dist_crop ) cloud_corner->push_back(pt);
        }

        PolygonMesh polymesh;
        ConvexHull<PointXYZRGB> chull;
        chull.setDimension(2);
        chull.setInputCloud(cloud_corner);
        chull.reconstruct (polymesh);    
        fromPCLPointCloud2(polymesh.cloud, *cloud_corner);

        float dist_min = INFINITY;
        int l_corner = -1;
        vector<uint32_t> &vert = polymesh.polygons[0].vertices;

        for( int l=0; l<vert.size(); l++ )
        {
            PointXYZRGB &pt = cloud_corner->points[vert[l]];
            float dist = compute_dist(pt_corner,pt);
            if( dist_min > dist )
            {
                dist_min = dist;
                l_corner = l;
            }
        }    
        
        int l_beg = l_corner;
        vector<int> incr{-1,1};    
        vector<Eigen::Vector3f> vecs(4);
        for( int i=0; i<2; i++ )
        {       
            vecs[i] << 0,0,0;

            float dist_stop = dist_crop*0.8;
            float dist_travel = 0;        
            int l_cur = l_beg;        
            while( dist_travel < dist_stop )
            {
                int l_nxt = (l_cur + incr[i] + vert.size()) % vert.size();
                PointXYZRGB pt_cur = cloud_corner->points[vert[l_cur]];
                PointXYZRGB pt_nxt = cloud_corner->points[vert[l_nxt]];
                float dist = compute_dist(pt_corner,pt_nxt);
                
                if( dist >= 0.01 )
                {
                    dist_travel += dist;
                    vecs[i][0] += (pt_nxt.x - pt_cur.x);
                    vecs[i][1] += (pt_nxt.y - pt_cur.y);
                    vecs[i][2] += (pt_nxt.z - pt_cur.z);
                }                            

                l_cur = l_nxt;            
            }
            vecs[i].normalize();
        }

        if( abs(vecs[0].dot(vecs[1])) > COS_75 )
        {
            cout << "Not a rectangle" << endl;
            return false;
        } 

        vector<pair<float,float> > lengths;
        lengths.push_back(pair<float,float>(boxinfo.width,  boxinfo.height));
        lengths.push_back(pair<float,float>(boxinfo.height, boxinfo.width ));
        for( int i=0; i<lengths.size(); i++ )
        {
            // Find the 3 points to cut
            vector<PointXYZRGB> pts_cut(3);
            pts_cut[0].x = pt_corner.x + vecs[0][0]*lengths[i].first;
            pts_cut[0].y = pt_corner.y + vecs[0][1]*lengths[i].first;
            pts_cut[0].z = pt_corner.z + vecs[0][2]*lengths[i].first;
            pts_cut[1].x = pt_corner.x + vecs[1][0]*lengths[i].second;
            pts_cut[1].y = pt_corner.y + vecs[1][1]*lengths[i].second;
            pts_cut[1].z = pt_corner.z + vecs[1][2]*lengths[i].second;
        
            PointXYZRGB pt1, pt2, pt3;
            pt1.x = pts_cut[0].x + pts_cut[1].x - pt_corner.x;
            pt1.y = pts_cut[0].y + pts_cut[1].y - pt_corner.y;
            pt1.z = pts_cut[0].z + pts_cut[1].z - pt_corner.z;

            Eigen::Vector3f vec1 = vecs[0].cross(vec_normal);
            vec1.normalize();
            if( vec1.dot(vecs[1]) < 0) vec1 = -vec1;
            pt2.x = pts_cut[0].x + vec1[0] * lengths[i].second;
            pt2.y = pts_cut[0].y + vec1[1] * lengths[i].second;
            pt2.z = pts_cut[0].z + vec1[2] * lengths[i].second;

            Eigen::Vector3f vec0 = vecs[1].cross(vec_normal);
            vec0.normalize();
            if( vec0.dot(vecs[0]) < 0) vec0 = -vec0;
            pt3.x = pts_cut[1].x + vec0[0] * lengths[i].first;
            pt3.y = pts_cut[1].y + vec0[1] * lengths[i].first;
            pt3.z = pts_cut[1].z + vec0[2] * lengths[i].first;

            pts_cut[2].x = (pt1.x + pt2.x + pt3.x) / 3.0;
            pts_cut[2].y = (pt1.y + pt2.y + pt3.y) / 3.0;
            pts_cut[2].z = (pt1.z + pt2.z + pt3.z) / 3.0;

#if 0
        {
            int v1,v2,v3;
            visualization::PCLVisualizer viewer;        
            viewer.setWindowName("debug");
            viewer.createViewPort(0.00,0.0,0.33,1.0,v1);
            viewer.createViewPort(0.33,0.0,0.66,1.0,v2);
            viewer.createViewPort(0.66,0.0,0.99,1.0,v3);
            viewer.setSize(1280,480);
            viewer.setPosition(0,0);
            viewer.setCameraPosition(2,0,1,0.5,0,-0.2,0,0,1);
            viewer.setBackgroundColor (0.2,0.2,0.2);
            viewer.addCoordinateSystem(0.1);
            viewer.addPointCloud(cloud_face,"cloud_face",v1);
            viewer.addPointCloud(cloud_prj,"cloud_prj",v2);
            viewer.addPointCloud(cloudvox_prj,"cloudvox_prj",v3);         

            viewer.addPointCloud(cloud_corner,"cloud_corner",v3);         
            viewer.setPointCloudRenderingProperties(
                    visualization::PCL_VISUALIZER_COLOR, 1,1,0,                
                    "cloud_corner",v3);

            pt_corner.r  = 255; pt_corner.g  = 255; pt_corner.b  = 255;
            pts_cut[0].r =   0; pts_cut[0].g =   0; pts_cut[0].b = 255;
            pts_cut[1].r =   0; pts_cut[1].g = 255; pts_cut[1].b =   0;
            pts_cut[2].r =   0; pts_cut[2].g = 255; pts_cut[2].b = 255;

            PointCloud<PointXYZRGB>::Ptr cloud_keys(new PointCloud<PointXYZRGB>);        
            cloud_keys->push_back(pt_corner);
            cloud_keys->push_back(pts_cut[0]);
            cloud_keys->push_back(pts_cut[1]);
            cloud_keys->push_back(pts_cut[2]);
            viewer.addPointCloud(cloud_keys,"cloud_keys",v3); 

            for( int ii=0; ii<2; ii++ )
            {            
                float dist_stop = dist_crop*0.8;
                float dist_travel = 0;        
                int l_cur = l_beg;        
                while( dist_travel < dist_stop )
                {
                    int l_nxt = (l_cur + incr[ii] + vert.size()) % vert.size();
                    PointXYZRGB pt_cur = cloud_corner->points[vert[l_cur]];
                    PointXYZRGB pt_nxt = cloud_corner->points[vert[l_nxt]];
                    float dist = compute_dist(pt_corner,pt_cur);                
                    dist_travel += dist;

                    stringstream ss;
                    ss << l_cur << "->" << l_nxt;                
                    viewer.addLine(pt_cur, pt_nxt, 
                        utils::colors_vis[ii][0]/255.,
                        utils::colors_vis[ii][1]/255.,
                        utils::colors_vis[ii][2]/255.,
                        "line_" + ss.str(), v3);

                    l_cur = l_nxt;            
                }
            }            
            
            viewer.spin();
        }
#endif

            vecs[2] << pts_cut[1].x-pts_cut[2].x, 
                       pts_cut[1].y-pts_cut[2].y, 
                       pts_cut[1].z-pts_cut[2].z;
            vecs[2].normalize();

            vecs[3] << pts_cut[0].x-pts_cut[2].x, 
                       pts_cut[0].y-pts_cut[2].y, 
                       pts_cut[0].z-pts_cut[2].z;
            vecs[3].normalize();

            Eigen::Vector3f vec2 = vecs[3].cross(vec_normal);
            vec2.normalize();
            if( vec2.dot(vecs[2]) < 0) vec2 = -vec2;

            Eigen::Vector3f vec3 = vecs[2].cross(vec_normal);
            vec3.normalize();
            if( vec3.dot(vecs[3]) < 0) vec3 = -vec3;
            
            // cut the points
            vector<ModelCoefficients> coef_cut(4);        
            GetPlane(vec0[0],vec0[1],vec0[2],pt_corner,coef_cut[0]);
            GetPlane(vec1[0],vec1[1],vec1[2],pt_corner,coef_cut[1]);
            GetPlane(vec2[0],vec2[1],vec2[2],pts_cut[2],coef_cut[2]);
            GetPlane(vec3[0],vec3[1],vec3[2],pts_cut[2],coef_cut[3]);

            PointCloud<PointXYZRGB>::Ptr cloud_cut(new PointCloud<PointXYZRGB>);
            vector<PointCloud<PointXYZRGB> > clouds_other(8);
            for( int p=0; p<cloud_prj->size(); p++ )
            {
                vector<float> dists(4);
                dists[0] = compute_dist(coef_cut[0],cloud_prj->points[p]);
                dists[1] = compute_dist(coef_cut[2],cloud_prj->points[p]);
                dists[2] = compute_dist(coef_cut[1],cloud_prj->points[p]);
                dists[3] = compute_dist(coef_cut[3],cloud_prj->points[p]);

                if(      dists[0]>0 && dists[1]>0 && dists[2]>0 && dists[3]>0 )
                    cloud_cut->push_back(cloud_face->points[p]);
                else if( dists[0]>0 && dists[1]>0 && dists[2]>0 && dists[3]<0 )
                    clouds_other[0].push_back(cloud_face->points[p]);
                else if( dists[0]>0 && dists[1]>0 && dists[2]<0 && dists[3]>0 )
                    clouds_other[1].push_back(cloud_face->points[p]);
                else if( dists[0]>0 && dists[1]<0 && dists[2]>0 && dists[3]>0 )
                    clouds_other[2].push_back(cloud_face->points[p]);
                else if( dists[0]>0 && dists[1]<0 && dists[2]>0 && dists[3]<0 )
                    clouds_other[3].push_back(cloud_face->points[p]);
                else if( dists[0]>0 && dists[1]<0 && dists[2]<0 && dists[3]>0 )
                    clouds_other[4].push_back(cloud_face->points[p]);
                else if( dists[0]<0 && dists[1]>0 && dists[2]>0 && dists[3]>0 )
                    clouds_other[5].push_back(cloud_face->points[p]);
                else if( dists[0]<0 && dists[1]>0 && dists[2]>0 && dists[3]<0 )
                    clouds_other[6].push_back(cloud_face->points[p]);
                else if( dists[0]<0 && dists[1]>0 && dists[2]<0 && dists[3]>0 )
                    clouds_other[7].push_back(cloud_face->points[p]);                    
            }

            PointCloud<PointXYZRGB>::Ptr cloud_cut_vox(new PointCloud<PointXYZRGB>);        
            vector<PointCloud<PointXYZRGB> > clouds_other_vox(8);
            for( int p=0; p<cloudvox_prj->size(); p++ )
            {
                vector<float> dists(4);
                dists[0] = compute_dist(coef_cut[0],cloudvox_prj->points[p]);
                dists[1] = compute_dist(coef_cut[2],cloudvox_prj->points[p]);
                dists[2] = compute_dist(coef_cut[1],cloudvox_prj->points[p]);
                dists[3] = compute_dist(coef_cut[3],cloudvox_prj->points[p]);

                if(      dists[0]>0 && dists[1]>0 && dists[2]>0 && dists[3]>0 )
                    cloud_cut_vox->push_back(cloudvox_prj->points[p]);
                else if( dists[0]>0 && dists[1]>0 && dists[2]>0 && dists[3]<0 )
                    clouds_other_vox[0].push_back(cloudvox_prj->points[p]);
                else if( dists[0]>0 && dists[1]>0 && dists[2]<0 && dists[3]>0 )
                    clouds_other_vox[1].push_back(cloudvox_prj->points[p]);
                else if( dists[0]>0 && dists[1]<0 && dists[2]>0 && dists[3]>0 )
                    clouds_other_vox[2].push_back(cloudvox_prj->points[p]);
                else if( dists[0]>0 && dists[1]<0 && dists[2]>0 && dists[3]<0 )
                    clouds_other_vox[3].push_back(cloudvox_prj->points[p]);
                else if( dists[0]>0 && dists[1]<0 && dists[2]<0 && dists[3]>0 )
                    clouds_other_vox[4].push_back(cloudvox_prj->points[p]);
                else if( dists[0]<0 && dists[1]>0 && dists[2]>0 && dists[3]>0 )
                    clouds_other_vox[5].push_back(cloudvox_prj->points[p]);
                else if( dists[0]<0 && dists[1]>0 && dists[2]>0 && dists[3]<0 )
                    clouds_other_vox[6].push_back(cloudvox_prj->points[p]);
                else if( dists[0]<0 && dists[1]>0 && dists[2]<0 && dists[3]>0 )
                    clouds_other_vox[7].push_back(cloudvox_prj->points[p]);                    
            }
            
            if( cloud_cut_vox->size()*resolution_sq > area_thresh )        
            {            
                PointCloud<PointXYZRGB>::Ptr cloud_other(new PointCloud<PointXYZRGB>);
                for( int o=0; o<clouds_other_vox.size(); o++ )
                {   
                    if( clouds_other_vox[o].size() > thresh )
                    {
                        *cloud_other += clouds_other[o];
                    }
                }
#if 0
                {            
                    int v1,v2,v3;
                    visualization::PCLVisualizer viewer;        
                    viewer.setWindowName("debug");
                    viewer.createViewPort(0.00,0.0,0.50,1.0,v1);
                    viewer.createViewPort(0.50,0.0,1.00,1.0,v2);
                    viewer.setSize(1280,480);
                    viewer.setPosition(0,0);
                    viewer.setCameraPosition(2,0,1,0.5,0,-0.2,0,0,1);
                    viewer.setBackgroundColor (0.2,0.2,0.2);
                    viewer.addCoordinateSystem(0.1);
                    viewer.addPointCloud(cloud_cut,"cloud_cut",v1);            
                    viewer.setPointCloudRenderingProperties(
                        visualization::PCL_VISUALIZER_COLOR, 1,0,0,
                        "cloud_cut",v1);
                    viewer.addPointCloud(cloud_other,"cloud_other",v1);
                    viewer.setPointCloudRenderingProperties(
                        visualization::PCL_VISUALIZER_COLOR, 0,0,1,
                        "cloud_other",v1);            
                    
                    
                    for( int ii=0; ii<2; ii++ )
                    {            
                        float dist_stop = dist_crop*0.9;
                        float dist_travel = 0;
                        float dist = -1;
                        int l_cur = l_beg;
                        while( dist < dist_stop )
                        {
                            int l_nxt = (l_cur + incr[ii] + vert.size()) % vert.size();

                            PointXYZRGB pt_cur = cloud_corner->points[vert[l_cur]];
                            PointXYZRGB pt_nxt = cloud_corner->points[vert[l_nxt]];

                            dist = compute_dist(pt_corner,pt_cur);

                            stringstream ss;
                            ss << l_cur << "->" << l_nxt;
                            viewer.addLine(pt_cur, pt_nxt, 1,1,1, "line_" + ss.str(), v2);

                            l_cur = l_nxt;
                        }
                    }            

                    pt_corner.r  = 255; pt_corner.g  = 255; pt_corner.b  = 255;
                    pts_cut[0].r =   0; pts_cut[0].g =   0; pts_cut[0].b = 255;
                    pts_cut[1].r =   0; pts_cut[1].g = 255; pts_cut[1].b =   0;
                    pts_cut[2].r =   0; pts_cut[2].g = 255; pts_cut[2].b = 255;

                    PointCloud<PointXYZRGB>::Ptr cloud_keys(new PointCloud<PointXYZRGB>);        
                    cloud_keys->push_back(pt_corner);
                    cloud_keys->push_back(pts_cut[0]);
                    cloud_keys->push_back(pts_cut[1]);
                    cloud_keys->push_back(pts_cut[2]);
                    transformPointCloud(*cloud_keys,*cloud_keys,tf_zrj4x4_inv);

                    viewer.addPointCloud(cloud_keys, "cloud_keys", v2);            

                    viewer.addPointCloud(cloud_prj,"cloud_prj",v2);
                    viewer.addPlane(coef_cut[0],"cut11",v2);
                    viewer.addPlane(coef_cut[1],"cut12",v2);
                    viewer.addPlane(coef_cut[2],"cut21",v2);
                    viewer.addPlane(coef_cut[3],"cut22",v2);
                    viewer.spin();
                }
#endif
                int idx_beg = clouds_res.size();
                float err = abs(cloud_cut_vox->size()*resolution_sq - boxinfo.area);
                bool succ
                 = Split_helper<PointT>( clouds_res, error + err, 
                                 cloud_other, vec_normal, boxinfo, resolution );
                if( succ )
                {
                    //cout << "Success" << endl;

                    int idx_end = clouds_res.size();

                    //cout << "beg: " << idx_beg << " "
                    //     << "end: " << idx_end << endl;

                    bool is_enough = false;
                    for( int r=idx_beg; r<idx_end; r++ )
                    {                        
                        if( clouds_res[r].first < resolution ) is_enough = true;
                        clouds_res[r].second.push_back(cloud_cut);
                    }

                    if( is_enough ) return true;
                    if( idx_end > 20 ) return true; // cut enough

                    succ_all = true;
                }
            }
        }
    }   

    return succ_all;
}

template<typename PointT>
bool Seg2DBox::Split( std::vector<typename pcl::PointCloud<PointT>::Ptr> &res,
                      const typename pcl::PointCloud<PointT>::Ptr cloud,
                      const Eigen::Vector3f &vec_normal,
                      const boxinfo_t &boxinfo,
                      const float resolution )
{
    vector<pair<float,vector<typename PointCloud<PointT>::Ptr> > > clouds_res;
    Split_helper<PointT>( clouds_res, 0, cloud, vec_normal, boxinfo, resolution);

    if( clouds_res.size()==0 ) return false;
    else
    {
        sort(clouds_res.begin(), clouds_res.end());        
        for( int r=0; r<clouds_res[0].second.size(); r++ )
        {                
            res.push_back(clouds_res[0].second[r]);
        }
        return true;
    }
}


template
bool Seg2DBox::Split_helper<PointXYZRGB>( 
    vector<pair<float,vector<pcl::PointCloud<PointXYZRGB>::Ptr> > > &clouds_res,                
    float error,
    const pcl::PointCloud<PointXYZRGB>::Ptr cloud_face,
    const Eigen::Vector3f &vec_normal,
    const boxinfo_t &boxinfo,
    const float resolution                        );

template
bool Seg2DBox::Split<PointXYZRGB>( 
                      std::vector<pcl::PointCloud<PointXYZRGB>::Ptr> &res,
                      const pcl::PointCloud<PointXYZRGB>::Ptr cloud,
                      const Eigen::Vector3f &vec_normal,
                      const boxinfo_t &boxinfo,
                      const float resolution );