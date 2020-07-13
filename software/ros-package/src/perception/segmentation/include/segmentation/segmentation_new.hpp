#ifndef SEGMENTATION_NEW__HPP__
#define SEGMENTATION_NEW__HPP__

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Segmentation 
{
public:
    Segmentation();
    ~Segmentation();

    void Segment( const cv::Mat &image, 
                  const cv::Mat &depth, 
                  const float depth_scale, 
                  const std::vector<float> &cam_in,
                  cv::Mat* label                    );

private:

    std::vector<float> cam_ex_eye;
};

typedef class Segmentation Segmentation;

#endif