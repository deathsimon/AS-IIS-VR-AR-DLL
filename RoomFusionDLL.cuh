#include <cuda_runtime.h>
// 用以檢查兩個浮點數是否十分接近
#define nearlyEqual(a,b) ( a - b < 0.005f && a - b > -0.005f )

bool InitCUDA();
void runGPUApplyDepth(unsigned char* image, float* depth, int imageWidth, int imageHeight, float threshold);
void runGPUApplyCorrection(float* depth, int imageWidth, int imageHeight,
	float left_slope, float left_inter, float left_p1x, float left_p1y, float left_p2x, float left_p2y,
	float right_slope, float right_inter, float right_p1x, float right_p1y, float right_p2x, float right_p2y,
	float top_slope, float top_inter, float top_p1x, float top_p1y, float top_p2x, float top_p2y,
	float down_slope, float down_inter, float down_p1x, float down_p1y, float down_p2x, float down_p2y
	);
void runGPUDepthShift(float* dst, float* src, int imageWidth, int imageHeight);

__global__ void gpuDepthShift(float* dst, float* src, int imageWidth, int imageHeight);
__global__ void gpuUpdateAlpha(unsigned char* img, int imageWidth, int imageHeight);
__global__ void gpuApplyDepth(unsigned char* image, float* depth, int imageWidth, int imageHeight, float threshold);
__global__ void gpuApplyCorrection(float* depth, int imageWidth, int imageHeight,
	float left_slope, float left_inter, float left_p1x, float left_p1y, float left_p2x, float left_p2y,
	float right_slope, float right_inter, float right_p1x, float right_p1y, float right_p2x, float right_p2y,
	float top_slope, float top_inter, float top_p1x, float top_p1y, float top_p2x, float top_p2y,
	float down_slope, float down_inter, float down_p1x, float down_p1y, float down_p2x, float down_p2y
	);
__device__ bool gpuIsRightSide(float px, float py, float slope, float yIntercept, float p1x, float p1y, float p2x, float p2y);
__device__ bool gpuIsLeftSide(float px, float py, float slope, float yIntercept, float p1x, float p1y, float p2x, float p2y);
__device__ bool gpuIsUpSide(float px, float py, float slope, float yIntercept, float p1x, float p1y, float p2x, float p2y);
__device__ bool gpuIsDownSide(float px, float py, float slope, float yIntercept, float p1x, float p1y, float p2x, float p2y);