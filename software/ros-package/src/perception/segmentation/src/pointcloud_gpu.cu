
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <curand.h>
#include <curand_kernel.h>

#include <omp.h>

#include "segmentation/pointcloud_gpu.cuh"

#define DEBUG 1

namespace pc_gpu
{

#define CUDA_CHECK(condition) \
{ \
    cudaError_t error = condition; \
    if( error != cudaSuccess ) \
    { \
        fprintf(stderr,"[Error] %s - %s:%d\n",cudaGetErrorString(error),__FILE__,__LINE__); \
        return -1; \
    } \
}

// original source: http://www.public.iastate.edu/~dicook/JSS/paper/code/svd.c
// simply modified for cuda device function calls
/* 
 * svdcomp - SVD decomposition routine. 
 * Takes an mxn matrix a and decomposes it into udv, where u,v are
 * left and right orthogonal transformation matrices, and d is a 
 * diagonal matrix of singular values.
 *
 * This routine is adapted from svdecomp.c in XLISP-STAT 2.1 which is 
 * code from Numerical Recipes adapted by Luke Tierney and David Betz.
 *
 * Input to dsvd is as follows:
 *   a = mxn matrix to be decomposed, gets overwritten with u
 *   m = row dimension of a
 *   n = column dimension of a
 *   w = returns the vector of singular values of a
 *   v = returns the right orthogonal transformation matrix
*/

/************************************************************
 *                                                          *
 *  Permission is hereby granted  to  any  individual   or  *
 *  institution   for  use,  copying, or redistribution of  *
 *  this code and associated documentation,  provided       *
 *  that   such  code  and documentation are not sold  for  *
 *  profit and the  following copyright notice is retained  *
 *  in the code and documentation:                          *
 *     Copyright (c) held by Dianne Cook                    *
 *  All Rights Reserved.                                    *
 *                                                          *
 *  Questions and comments are welcome, and I request       *
 *  that you share any modifications with me.               *
 *                                                          *
 *                Dianne Cook                               *
 *             dicook@iastate.edu                           *
 *                                                          *
 ************************************************************/

#define PRECISION1 32768
#define PRECISION2 16384
/*#define PI 3.1415926535897932*/
#define MIN(x,y) ( (x) < (y) ? (x) : (y) )
#define MAX(x,y) ((x)>(y)?(x):(y))
#define SIGN(a, b) ((b) >= 0.0 ? fabs(a) : -fabs(a))
#define MAXINT 2147483647
#define ASCII_TEXT_BORDER_WIDTH 4
#define MAXHIST 100
#define STEP0 0.01
#define FORWARD 1
#define BACKWARD -1
#define PROJ_DIM 5
#define True 1
#define False 0

typedef struct {
    float x, y, z;
} fcoords;

typedef struct {
    long x, y, z;
} lcoords;

typedef struct {
    int x, y, z;
} icoords;

typedef struct {
    float min, max;
} lims;


/* grand tour history */
typedef struct hist_rec {
  struct hist_rec *prev, *next;
  float *basis[3];
  int pos;
} hist_rec;


__device__ __forceinline__ double PYTHAG(double a, double b)
{
    double at = fabs(a), bt = fabs(b), ct, result;

    if (at > bt)       { ct = bt / at; result = at * sqrt(1.0 + ct * ct); }
    else if (bt > 0.0) { ct = at / bt; result = bt * sqrt(1.0 + ct * ct); }
    else result = 0.0;
    return(result);
}

__device__  int dsvd(float **a, int m, int n, float *w, float **v)
{
    int flag, i, its, j, jj, k, l, nm;
    double c, f, h, s, x, y, z;
    double anorm = 0.0, g = 0.0, scale = 0.0;
    double *rv1;
  
    if (m < n) 
    {
        printf("#rows must be > #cols \n");
        return(0);
    }
  
    rv1 = (double *)malloc((unsigned int) n*sizeof(double));

/* Householder reduction to bidiagonal form */
    for (i = 0; i < n; i++) 
    {
        /* left-hand reduction */
        l = i + 1;
        rv1[i] = scale * g;
        g = s = scale = 0.0;
        if (i < m) 
        {
            for (k = i; k < m; k++) 
                scale += fabs((double)a[k][i]);
            if (scale) 
            {
                for (k = i; k < m; k++) 
                {
                    a[k][i] = (float)((double)a[k][i]/scale);
                    s += ((double)a[k][i] * (double)a[k][i]);
                }
                f = (double)a[i][i];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i][i] = (float)(f - g);
                if (i != n - 1) 
                {
                    for (j = l; j < n; j++) 
                    {
                        for (s = 0.0, k = i; k < m; k++) 
                            s += ((double)a[k][i] * (double)a[k][j]);
                        f = s / h;
                        for (k = i; k < m; k++) 
                            a[k][j] += (float)(f * (double)a[k][i]);
                    }
                }
                for (k = i; k < m; k++) 
                    a[k][i] = (float)((double)a[k][i]*scale);
            }
        }
        w[i] = (float)(scale * g);
    
        /* right-hand reduction */
        g = s = scale = 0.0;
        if (i < m && i != n - 1) 
        {
            for (k = l; k < n; k++) 
                scale += fabs((double)a[i][k]);
            if (scale) 
            {
                for (k = l; k < n; k++) 
                {
                    a[i][k] = (float)((double)a[i][k]/scale);
                    s += ((double)a[i][k] * (double)a[i][k]);
                }
                f = (double)a[i][l];
                g = -SIGN(sqrt(s), f);
                h = f * g - s;
                a[i][l] = (float)(f - g);
                for (k = l; k < n; k++) 
                    rv1[k] = (double)a[i][k] / h;
                if (i != m - 1) 
                {
                    for (j = l; j < m; j++) 
                    {
                        for (s = 0.0, k = l; k < n; k++) 
                            s += ((double)a[j][k] * (double)a[i][k]);
                        for (k = l; k < n; k++) 
                            a[j][k] += (float)(s * rv1[k]);
                    }
                }
                for (k = l; k < n; k++) 
                    a[i][k] = (float)((double)a[i][k]*scale);
            }
        }
        anorm = MAX(anorm, (fabs((double)w[i]) + fabs(rv1[i])));
    }
  
    /* accumulate the right-hand transformation */
    for (i = n - 1; i >= 0; i--) 
    {
        if (i < n - 1) 
        {
            if (g) 
            {
                for (j = l; j < n; j++)
                    v[j][i] = (float)(((double)a[i][j] / (double)a[i][l]) / g);
                    /* double division to avoid underflow */
                for (j = l; j < n; j++) 
                {
                    for (s = 0.0, k = l; k < n; k++) 
                        s += ((double)a[i][k] * (double)v[k][j]);
                    for (k = l; k < n; k++) 
                        v[k][j] += (float)(s * (double)v[k][i]);
                }
            }
            for (j = l; j < n; j++) 
                v[i][j] = v[j][i] = 0.0;
        }
        v[i][i] = 1.0;
        g = rv1[i];
        l = i;
    }
  
    /* accumulate the left-hand transformation */
    for (i = n - 1; i >= 0; i--) 
    {
        l = i + 1;
        g = (double)w[i];
        if (i < n - 1) 
            for (j = l; j < n; j++) 
                a[i][j] = 0.0;
        if (g) 
        {
            g = 1.0 / g;
            if (i != n - 1) 
            {
                for (j = l; j < n; j++) 
                {
                    for (s = 0.0, k = l; k < m; k++) 
                        s += ((double)a[k][i] * (double)a[k][j]);
                    f = (s / (double)a[i][i]) * g;
                    for (k = i; k < m; k++) 
                        a[k][j] += (float)(f * (double)a[k][i]);
                }
            }
            for (j = i; j < m; j++) 
                a[j][i] = (float)((double)a[j][i]*g);
        }
        else 
        {
            for (j = i; j < m; j++) 
                a[j][i] = 0.0;
        }
        ++a[i][i];
    }

    /* diagonalize the bidiagonal form */
    for (k = n - 1; k >= 0; k--) 
    {                             /* loop over singular values */
        for (its = 0; its < 30; its++) 
        {                         /* loop over allowed iterations */
            flag = 1;
            for (l = k; l >= 0; l--) 
            {                     /* test for splitting */
                nm = l - 1;
                if (fabs(rv1[l]) + anorm == anorm) 
                {
                    flag = 0;
                    break;
                }
                if (fabs((double)w[nm]) + anorm == anorm) 
                    break;
            }
            if (flag) 
            {
                c = 0.0;
                s = 1.0;
                for (i = l; i <= k; i++) 
                {
                    f = s * rv1[i];
                    if (fabs(f) + anorm != anorm) 
                    {
                        g = (double)w[i];
                        h = PYTHAG(f, g);
                        w[i] = (float)h; 
                        h = 1.0 / h;
                        c = g * h;
                        s = (- f * h);
                        for (j = 0; j < m; j++) 
                        {
                            y = (double)a[j][nm];
                            z = (double)a[j][i];
                            a[j][nm] = (float)(y * c + z * s);
                            a[j][i] = (float)(z * c - y * s);
                        }
                    }
                }
            }
            z = (double)w[k];
            if (l == k) 
            {                  /* convergence */
                if (z < 0.0) 
                {              /* make singular value nonnegative */
                    w[k] = (float)(-z);
                    for (j = 0; j < n; j++) 
                        v[j][k] = (-v[j][k]);
                }
                break;
            }
            if (its >= 30) {
                free((void*) rv1);
                printf("No convergence after 30,000! iterations \n");
                return(0);
            }
    
            /* shift from bottom 2 x 2 minor */
            x = (double)w[l];
            nm = k - 1;
            y = (double)w[nm];
            g = rv1[nm];
            h = rv1[k];
            f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = PYTHAG(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + SIGN(g, f))) - h)) / x;
          
            /* next QR transformation */
            c = s = 1.0;
            for (j = l; j <= nm; j++) 
            {
                i = j + 1;
                g = rv1[i];
                y = (double)w[i];
                h = s * g;
                g = c * g;
                z = PYTHAG(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y = y * c;
                for (jj = 0; jj < n; jj++) 
                {
                    x = (double)v[jj][j];
                    z = (double)v[jj][i];
                    v[jj][j] = (float)(x * c + z * s);
                    v[jj][i] = (float)(z * c - x * s);
                }
                z = PYTHAG(f, h);
                w[j] = (float)z;
                if (z) 
                {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = (c * g) + (s * y);
                x = (c * y) - (s * g);
                for (jj = 0; jj < m; jj++) 
                {
                    y = (double)a[jj][j];
                    z = (double)a[jj][i];
                    a[jj][j] = (float)(y * c + z * s);
                    a[jj][i] = (float)(z * c - y * s);
                }
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = (float)x;
        }
    }
    free((void*) rv1);
    return(1);
}

void Depth2PointCloud( const uint16_t* depth, int width, int height, 
                       const float* cam_intrinsic, float depth_scale,
                       pc_gpu::Point* points )
{
#pragma omp parallel 
    for( int r=0; r<height; r++ )
    for( int c=0; c<width;  c++ )
    {
        int idx = r*width + c;

        pc_gpu::Point* pt = &(points[idx]);
        memset(pt,0,sizeof(pc_gpu::Point));

        // 2D -> 3D (camera coordinate)
        float pt_z = ((float)depth[idx]) * depth_scale;
        if( pt_z < 0.10 ) continue;    

        float pt_x = (((float)c) - cam_intrinsic[2]) / cam_intrinsic[0] * pt_z;
        float pt_y = (((float)r) - cam_intrinsic[5]) / cam_intrinsic[4] * pt_z;
        
        pt->x=pt_x; pt->y=pt_y; pt->z=pt_z;

        int size_neibors = 0;
        for( int x=c-SIZE_WINDOW; x<c+SIZE_WINDOW; x++ )
        for( int y=r-SIZE_WINDOW; y<r+SIZE_WINDOW; y++ )
        {
            if( x<0 || y<0 || x>=width || y>=height ) continue;
            if( x==c && y==r ) continue;

            int i = y*width + x;
            pt->idxes_neibors[size_neibors] = i;
            size_neibors++;
        }
        pt->size_neibors = size_neibors;
    }
}

void GetValidPointList( pc_gpu::Point* points, int len_points,
                        int* idxes_valid, int* len_ret             )
{
    int n=0;
    for( int i=0; i<len_points; i++ )
    {
        if( points[i].z > 0 )
        {
            idxes_valid[n] = i;
            n++;
        }
    } 
    *len_ret = n;
}

void RemovePlane( pc_gpu::Point* points, 
                  int* idxes_valid, int len_valid_points, 
                  float* plane                            )
{
#pragma omp parallel 
    for( int i=0; i<len_valid_points; i++ )
    {
        pc_gpu::Point* pt = &points[idxes_valid[i]];
        if( pt->z > 0 )
        {
            float val = (pt->x-plane[3])*plane[0] + 
                        (pt->y-plane[4])*plane[1] + 
                        (pt->z-plane[5])*plane[2];

            if( abs(val) < 0.01 )
            {                
                pt->z = 0;
            }
        }
    }
}

__global__
void __FindPlane__( pc_gpu::Point* points,
                    int* idxes_valid,
                    int len, 
                    int* inliers, float* planes )
{
    if( len < 3 ) return;

    int idx = blockIdx.x*blockDim.x + threadIdx.x;
    if( idx >= len ) return;

    curandState_t state;      
    curand_init(idx, 0, 0, &state);

    int i1, i2, i3;
    do
    {
        i1 = curand(&state) % len;
        i2 = curand(&state) % len;
        i3 = curand(&state) % len;
    } while( i1==i2 || i2==i3 || i3==i1 );

    pc_gpu::Point* p1 = &points[idxes_valid[i1]];
    pc_gpu::Point* p2 = &points[idxes_valid[i2]];
    pc_gpu::Point* p3 = &points[idxes_valid[i3]];

    float a[3], b[3];
    a[0] = p1->x-p2->x;
    a[1] = p1->y-p2->y;
    a[2] = p1->z-p2->z;
    b[0] = p1->x-p3->x;
    b[1] = p1->y-p3->y;
    b[2] = p1->z-p3->z;

    // cross product
    float nx = a[1]*b[2] - a[2]*b[1];
    float ny = a[2]*b[0] - a[0]*b[2];
    float nz = a[0]*b[1] - a[1]*b[0];
    float norm = sqrt(nx*nx + ny*ny + nz*nz);

    nx /= norm;
    ny /= norm;
    nz /= norm;

    planes[idx*6 + 0] = nx;
    planes[idx*6 + 1] = ny;
    planes[idx*6 + 2] = nz;
    planes[idx*6 + 3] = p1->x;
    planes[idx*6 + 4] = p1->y;
    planes[idx*6 + 5] = p1->z;

    inliers[idx] = 0;
    for( int i=0; i<len; i++ )
    {
        pc_gpu::Point* pt_i = &points[idxes_valid[i]];

        float val = (pt_i->x-p1->x)*nx + 
                    (pt_i->y-p1->y)*ny + 
                    (pt_i->z-p1->z)*nz;

        if( abs(val) < 0.01 ) inliers[idx]++;
    }
}

__global__
void __FindNeibors__( pc_gpu::Point* points, int* idxes_valid, int len )
{
    int idx = blockIdx.x*blockDim.x + threadIdx.x;
    if( idx >= len ) return;

    pc_gpu::Point* pt = &points[idxes_valid[idx]];
    if( pt->z == 0 ) return; // will never happen but make sure

    int m = 0;
    const int n = 3;
    float** vecs = (float**)malloc(sizeof(float*)*MAX_NEIBORS);
    float** V = (float**)malloc(sizeof(float*)*n);
    for( int i_n=0; i_n<n; i_n++ ) V[i_n] = (float*)malloc(sizeof(float)*n);

    const float rad_nei = 0.01;
    const float rad_nei_sq = rad_nei * rad_nei;
    for( int i=0; i<pt->size_neibors; i++ )
    {
        int idx_nei = pt->idxes_neibors[i];
        if( idx_nei == -1 ) continue;
        
        pc_gpu::Point* pt_i = &points[idx_nei];        
        if( pt_i->z == 0 )
        {
            pt->idxes_neibors[i] = -1;
            continue;
        }

        float dist_sq = (pt->x-pt_i->x)*(pt->x-pt_i->x) +
                        (pt->y-pt_i->y)*(pt->y-pt_i->y) +
                        (pt->z-pt_i->z)*(pt->z-pt_i->z);
                
        if( dist_sq > rad_nei_sq || dist_sq == 0 || dist_sq < 0.000001 )
        {
            pt->idxes_neibors[i] = -1;
            continue;
        }

        float norm = sqrt(dist_sq);
        vecs[m] = (float*)malloc(sizeof(float)*n);
        vecs[m][0] = (pt->x-pt_i->x) / norm;
        vecs[m][1] = (pt->y-pt_i->y) / norm;
        vecs[m][2] = (pt->z-pt_i->z) / norm;

        m++;
    }

    if( m<n )    
    {
        // maybe outlier
        pt->x  = 0; pt->y  = 0; pt->z  = 0;
        pt->nx = 0; pt->ny = 0; pt->nz = 0;

        for( int i_n=0; i_n<n; i_n++ ) free(V[i_n]);
        free(V);
        for( int i_m=0; i_m<m; i_m++ ) free(vecs[i_m]);
        free(vecs);

        return;
    }

    float* w = (float*)malloc(sizeof(float)*m);

    dsvd(vecs, m, n, w, V);

    int i_w = 0;
    float w_min = w[0];
    for( int i_n=1; i_n<n; i_n++ )
    {
        if( w_min > w[i_n])
        {
            w_min = w[i_n];
            i_w = i_n;
        }
    }
    
    if( V[2][i_w] < 0 )
    {
        pt->nx =  V[0][i_w]; pt->ny =  V[1][i_w]; pt->nz =  V[2][i_w];
    }
    else
    {
        pt->nx = -V[0][i_w]; pt->ny = -V[1][i_w]; pt->nz = -V[2][i_w];
    }
    
    free(w);
    for( int i_n=0; i_n<n; i_n++ ) free(V[i_n]);
    free(V);
    for( int i_m=0; i_m<m; i_m++ ) free(vecs[i_m]);
    free(vecs);
}

void GetGridSizeBlockSize(int* gridSize, int* blockSize, int N)
{
    if( N > 65535 )
    {
        *gridSize = 65535;
        *blockSize = N / 65535 + 1;
    }
    else
    {
        *gridSize = N;
        *blockSize = 1;
    }
}

int  Compute_PointCloud( const int device, 
                         const uint16_t* depth, const int width, const int height,
                         const float* cam_intrinsic, const float depth_scale,
                         pc_gpu::Point* points, const bool remove_plane )
{
    int blockSize, gridSize;

#if DEBUG
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);
    float milliseconds = 0;
#endif

    CUDA_CHECK(cudaSetDevice(device));


    //////// Depth to PointCloud ////////
#if DEBUG
    cudaEventRecord(start);
    printf("----------------------------\n");
    printf("dev: %d\n", device);
    printf("depth: %d\n", depth[0]);
    printf("depth_scale: %f\n", depth_scale);
    printf("%d x %d\n", width, height);
    printf("%f %f %f\n%f %f %f\n%f %f %f\n", 
            cam_intrinsic[0],cam_intrinsic[1],cam_intrinsic[2],
            cam_intrinsic[3],cam_intrinsic[4],cam_intrinsic[5],
            cam_intrinsic[6],cam_intrinsic[7],cam_intrinsic[8] );
    printf("----------------------------\n");
#endif

    int len_points = width*height;
    Depth2PointCloud(depth, width, height, cam_intrinsic, depth_scale, points);
    
    pc_gpu::Point* points_GPU;
    size_t points_size = sizeof(pc_gpu::Point)*len_points;    
    CUDA_CHECK(cudaMalloc(&points_GPU, points_size));
    CUDA_CHECK(cudaMemcpy( points_GPU, points, points_size,
                           cudaMemcpyHostToDevice           ));

#if DEBUG
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    printf("Depth to Pointcloud %f ms\n", milliseconds);
#endif

    //////// Valid Point List ////////
#if DEBUG
    cudaEventRecord(start);
#endif

    int len_valid_points = 0;    
    int* idxes_valid = (int*)malloc(sizeof(int)*len_points);
    int* idxes_valid_GPU;
    CUDA_CHECK(cudaMalloc(&idxes_valid_GPU, sizeof(int)*len_points));

    GetValidPointList(points, len_points, idxes_valid, &len_valid_points);    
    CUDA_CHECK(cudaMemcpy( idxes_valid_GPU, idxes_valid, 
                           sizeof(int)*len_valid_points,
                           cudaMemcpyHostToDevice        ));

#if DEBUG
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    printf("Invalidate points %f ms\n", milliseconds);    
#endif

    if( remove_plane )
    {
        // Find Plane
#if DEBUG
        cudaEventRecord(start);
#endif

        int* n_inliers_GPU;
        float* planes_GPU;
        CUDA_CHECK(cudaMalloc(&n_inliers_GPU,sizeof(int)*(len_valid_points) ));
        CUDA_CHECK(cudaMalloc(&planes_GPU,sizeof(float)*(len_valid_points)*6));

        GetGridSizeBlockSize(&gridSize, &blockSize, 1000);

        __FindPlane__<<<200,5>>>( points_GPU,
                                  idxes_valid_GPU,
                                  len_valid_points, 
                                  n_inliers_GPU, 
                                  planes_GPU        );
        CUDA_CHECK(cudaGetLastError());
        cudaDeviceSynchronize();

#if DEBUG
        cudaEventRecord(stop);
        cudaEventSynchronize(stop);
        cudaEventElapsedTime(&milliseconds, start, stop);
        printf("Find Plane %f ms\n", milliseconds);

        cudaEventRecord(start);
#endif

        float plane[6];
        {
            int idx_max = 0;
            int* n_inliers = (int*)malloc(sizeof(int)*len_valid_points);
            CUDA_CHECK(cudaMemcpy( n_inliers, n_inliers_GPU, 
                                   sizeof(int)*len_valid_points, 
                                   cudaMemcpyDeviceToHost        ));
            int val_max = n_inliers[0];
            for( int i=1; i<len_valid_points; i++ )
            {
                if( val_max < n_inliers[i] )
                {
                    val_max = n_inliers[i];
                    idx_max = i;
                }
            }
            free(n_inliers);

            CUDA_CHECK(cudaMemcpy( plane, &planes_GPU[idx_max*6],
                       sizeof(float)*6, cudaMemcpyDeviceToHost ));
        }
        
#if DEBUG
        cudaEventRecord(stop);
        cudaEventSynchronize(stop);
        cudaEventElapsedTime(&milliseconds, start, stop);
        printf("Find Max %f ms\n", milliseconds);

        cudaEventRecord(start);
#endif
        // Remove Plane
        RemovePlane( points, idxes_valid, len_valid_points, plane );
        GetValidPointList(points, len_points, idxes_valid, &len_valid_points);    
        CUDA_CHECK(cudaMemcpy( idxes_valid_GPU, idxes_valid, 
                               sizeof(int)*len_valid_points,
                               cudaMemcpyHostToDevice        ));

        CUDA_CHECK(cudaMemcpy( points_GPU, points, points_size,
                               cudaMemcpyHostToDevice           ));

        CUDA_CHECK(cudaFree(n_inliers_GPU));
        CUDA_CHECK(cudaFree(planes_GPU));

#if DEBUG
        cudaEventRecord(stop);
        cudaEventSynchronize(stop);
        cudaEventElapsedTime(&milliseconds, start, stop);
        printf("Remove Plane %f ms\n", milliseconds);
#endif
    }

    // Compute Normal
#if DEBUG
    cudaEventRecord(start);
    printf("len: %d\n",len_valid_points);
#endif

    size_t dynamicSMemSize = (len_valid_points)*(MAX_NEIBORS*sizeof(float*)   + 
                                                 MAX_NEIBORS*sizeof(float )*3 + 
                                                 MAX_NEIBORS*sizeof(float )   +                                               
                                                           3*sizeof(float*)   +
                                                           3*sizeof(float )*3 +
                                                           3*sizeof(double)   +
                                                                         1024 );

    CUDA_CHECK(cudaThreadSetLimit( cudaLimitMallocHeapSize, dynamicSMemSize));

    GetGridSizeBlockSize(&gridSize, &blockSize, len_valid_points);
    __FindNeibors__<<<gridSize,blockSize>>>( points_GPU,
                                             idxes_valid_GPU,
                                             len_valid_points );

    CUDA_CHECK(cudaGetLastError());
    cudaDeviceSynchronize();

    CUDA_CHECK(cudaMemcpy( points, points_GPU, points_size,
                           cudaMemcpyDeviceToHost           ));

#if DEBUG
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);
    cudaEventElapsedTime(&milliseconds, start, stop);
    printf("Compute Normal %f ms\n", milliseconds);
#endif 

    CUDA_CHECK(cudaFree(idxes_valid_GPU));
    free(idxes_valid);

    CUDA_CHECK(cudaFree(points_GPU));

    cudaDeviceReset();

    return 1;
}

}
