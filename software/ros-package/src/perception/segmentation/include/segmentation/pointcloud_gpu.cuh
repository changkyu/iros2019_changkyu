#include <stdint.h>

namespace pc_gpu
{

struct PointNode_GPU;

#define SIZE_WINDOW (5)
#define MAX_NEIBORS ((SIZE_WINDOW*2+1)*(SIZE_WINDOW*2+1))

typedef struct Point
{
    float x;
    float y;
    float z;
    float nx;
    float ny;
    float nz;

    //struct Point_GPU* neibors[MAX_NEIBORS];    
    int idxes_neibors[MAX_NEIBORS];
    uint32_t size_neibors;
} Point;

//extern "C"
//{
int Compute_PointCloud( const int device, 
                         const uint16_t* depth, const int width, const int height,
                         const float* cam_intrinsic, const float depth_scale,
                         Point* points, const bool remove_plane );
//}

}
