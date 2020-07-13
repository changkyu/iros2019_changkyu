#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>

typedef class KNearestNeighbors
{
public:
    KNearestNeighbors()
    {
        knn = cv::ml::KNearest::create();
    }

    KNearestNeighbors( std::vector<Point>* pts_in,
                       std::vector<float> &K )
    {
        knn = cv::ml::KNearest::create();
        Train(pts_in, K);
    }
    ~KNearestNeighbors(){}

    void Train( std::vector<Point>* pts_in, 
                std::vector<float> &K    );

    void FindNearest( std::vector<Point> &pts_in,
                      std::vector<Point> &pts_out,
                      std::vector<double> &distances );
private:
    cv::Ptr<cv::ml::KNearest> knn;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* sv;
    cv::Mat labels;
    std::vector<float> camera_K;
    void Projection(pcl::PointXYZRGB &pt, float *x_prj, float *y_prj);
    
} KNearestNeighbors;

void KNearestNeighbors::Train( vector<PointCloud<PointXYZRGB>::Ptr>* sv_, 
                            vector<float> &K )
{
    sv = sv_;
    camera_K.resize(9);
    for( int i=0; i<9; i++ ) camera_K[i] = K[i];

    size_t n_cloud = 0;
    for( size_t d=0; d<sv->size(); d++ )
    {
        n_cloud += (*sv)[d]->size();
    }
    Mat pts2d(n_cloud,2,CV_32FC1); // [x,y]
    Mat idxes(n_cloud,1,CV_32FC1);
    labels = Mat(n_cloud,2,CV_32FC1);

    int idx_cloud = 0;
    for( size_t d=0; d<sv->size(); d++ )
    {
        // Create map 2D -> 3D point
        for( size_t p=0; p<(*sv)[d]->size(); p++ )
        {
            PointXYZRGB &pt = (*(*sv)[d])[p];
            float x,y;
            Projection(pt, &x,&y);
            int r=int(y+0.5), c=int(x+0.5);
            
            pts2d.at<float>(idx_cloud,0) = c;
            pts2d.at<float>(idx_cloud,1) = r;
            idxes.at<float>(idx_cloud,0) = idx_cloud;
            labels.at<float>(idx_cloud,0) = d;
            labels.at<float>(idx_cloud,1) = p;
            idx_cloud++;
        }        
    }
    knn->train(pts2d, ml::ROW_SAMPLE, idxes);
}

void KNearestNeighbors::FindNearest( vector<Point2f>     &pts2d,
                                  vector<PointXYZRGB> &pts3d,
                                  vector<int>         &idxes_sv )
{
    size_t n_pts = pts2d.size();    
    Mat mat2d(n_pts,2,CV_32FC1);
    for( size_t p=0; p<n_pts; p++ )
    {
        mat2d.at<float>(p,0) = pts2d[p].x;
        mat2d.at<float>(p,1) = pts2d[p].y;
    }

    Mat matIDX, dist;
    knn->findNearest(mat2d, 1, noArray(), matIDX, dist);
    pts3d.resize(n_pts);
    idxes_sv.resize(n_pts);
    for( size_t p=0; p<n_pts; p++ )
    {
        if( dist.at<float>(p,0) < 5 )
        {
            int i = (int)matIDX.at<float>(p,0);
            int idx_sv = labels.at<float>(i,0);
            int idx_pt = labels.at<float>(i,1);

            pts3d[p] = (*((*sv)[idx_sv]))[idx_pt];
            idxes_sv[p] = idx_sv;
        }
        else
        {
            pts3d[p].x = 0;
            pts3d[p].y = 0;
            pts3d[p].z = 0;
            idxes_sv[p] = -1;                
        }
    }   
}
