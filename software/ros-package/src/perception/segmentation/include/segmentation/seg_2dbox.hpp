#ifndef SEG_2DBOX__HPP__
#define SEG_2DBOX__HPP__

#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef class Seg2DBox
{
public:
    typedef struct
    {   
        float area;
        float width;
        float height;
    } boxinfo_t;

    Seg2DBox();
    ~Seg2DBox();

    template<typename PointT>    
    bool Split( std::vector<typename pcl::PointCloud<PointT>::Ptr> &res,
                const typename pcl::PointCloud<PointT>::Ptr cloud,
                const Eigen::Vector3f &vec_normal,
                const boxinfo_t &boxinfo,
                const float resolution=0.005 );
private:

    template<typename PointT>
    bool Split_helper( std::vector<std::pair<float,std::vector<
                         typename pcl::PointCloud<PointT>::Ptr> > > &clouds_res,                
                       float error,
                       const typename pcl::PointCloud<PointT>::Ptr cloud,
                       const Eigen::Vector3f &vec_normal,
                       const boxinfo_t &boxinfo,
                       const float resolution=0.005                     );
} Seg2DBox;

#endif