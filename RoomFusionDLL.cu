#include "stdio.h"
#include "RoomFusionDLL.cuh"

bool InitCUDA()
{
	int count;

	cudaGetDeviceCount(&count);
	if (count == 0) {
		fprintf(stderr, "There is no device.\n");
		return false;
	}

	int i;
	for (i = 0; i < count; i++) {
		cudaDeviceProp prop;
		if (cudaGetDeviceProperties(&prop, i) == cudaSuccess) {
			if (prop.major >= 1) {
				break;
			}
		}
	}

	if (i == count) {
		fprintf(stderr, "There is no device supporting CUDA 1.x.\n");
		return false;
	}

	cudaSetDevice(i);
	fprintf(stderr, "CUDA Ready!\n");



	return true;
}

void runGPUApplyDepth(unsigned char* image, float* depth, int imageWidth, int imageHeight, float threshold){
	dim3 dimBlock(imageHeight, 1);
	dim3 dimGrid(imageWidth, 1);
	gpuApplyDepth << <dimGrid, dimBlock >> >(image, depth, imageWidth, imageHeight, threshold);
}

void runGPUApplyCorrection(float* depth, int imageWidth, int imageHeight,
	float left_slope, float left_inter, float left_p1x, float left_p1y, float left_p2x, float left_p2y,
	float right_slope, float right_inter, float right_p1x, float right_p1y, float right_p2x, float right_p2y,
	float top_slope, float top_inter, float top_p1x, float top_p1y, float top_p2x, float top_p2y,
	float down_slope, float down_inter, float down_p1x, float down_p1y, float down_p2x, float down_p2y
	){
	dim3 dimBlock(imageHeight, 1);
	dim3 dimGrid(imageWidth, 1);
	gpuApplyCorrection << <dimGrid, dimBlock >> >(depth, imageWidth, imageHeight, 
		left_slope, left_inter, left_p1x, left_p1y, left_p2x, left_p2y,
		right_slope, right_inter, right_p1x, right_p1y, right_p2x, right_p2y,
		top_slope, top_inter, top_p1x, top_p1y, top_p2x, top_p2y,
		down_slope, down_inter, down_p1x, down_p1y, down_p2x, down_p2y
		);
}

__global__  void gpuApplyDepth(unsigned char* image, float* depth, int imageWidth, int imageHeight, float threshold){
	int w = blockIdx.x;
	int h = threadIdx.x;
	if (w < imageWidth && h < imageHeight){
		int positionIndex = h * imageWidth + w;
		int pixelIndex = positionIndex * 4;
		float depthVal = depth[positionIndex];
		if (depthVal > threshold){
			// change to invisible
			image[pixelIndex + 0] = 0;
			image[pixelIndex + 1] = 128;
			image[pixelIndex + 2] = 0;
			image[pixelIndex + 3] = 0;
		}
	}
}

__global__ void gpuApplyCorrection(float* depth, int imageWidth, int imageHeight,
	float left_slope, float left_inter, float left_p1x, float left_p1y, float left_p2x, float left_p2y,
	float right_slope, float right_inter, float right_p1x, float right_p1y, float right_p2x, float right_p2y,
	float top_slope, float top_inter, float top_p1x, float top_p1y, float top_p2x, float top_p2y,
	float down_slope, float down_inter, float down_p1x, float down_p1y, float down_p2x, float down_p2y
	)
{
	int w = blockIdx.x;
	int h = threadIdx.x;
	if (w < imageWidth && h < imageHeight){
		int positionIndex = h * imageWidth + w;
		int y = imageHeight - h - 1;
		if (
				gpuIsRightSide(w, y, left_slope, left_inter, left_p1x, left_p1y, left_p2x, left_p2y) &&
				gpuIsLeftSide(w, y, right_slope, right_inter, right_p1x, right_p1y, right_p2x, right_p2y) &&
				gpuIsDownSide(w, y, top_slope, top_inter, top_p1x, top_p1y, top_p2x, top_p2y) &&
				gpuIsUpSide(w, y, down_slope, down_inter, down_p1x, down_p1y, down_p2x, down_p2y)
			)
		{
			// keep depth
		}
		else{
			// no depth
			depth[positionIndex] = 0.0f;
		}
	}
}


__device__ bool gpuIsRightSide(float px, float py, float slope, float yIntercept, float p1x, float p1y, float p2x, float p2y){
	if (nearlyEqual(p2y, p1y)){ // horz
		return false;
	}
	else if (nearlyEqual(p2x, p1x)){ // vertical
		return px > p1x;
	}
	float cSolution = (slope*px) + yIntercept;
	if (py > cSolution){
		return p2x <= p1x;
	}
	else{
		return p2x > p1x;
	}
}
__device__ bool gpuIsUpSide(float px, float py, float slope, float yIntercept, float p1x, float p1y, float p2x, float p2y){
	if (nearlyEqual(p2x - p1x, 0)){ // vertical
		return false;
	}
	if (slope > 0){
		return gpuIsLeftSide(px, py, slope, yIntercept, p1x, p1y, p2x, p2y);
	}
	else{
		return gpuIsRightSide(px, py, slope, yIntercept, p1x, p1y, p2x, p2y);
	}
}
__device__ bool gpuIsLeftSide(float px, float py, float slope, float yIntercept, float p1x, float p1y, float p2x, float p2y){
	return !gpuIsRightSide(px, py, slope, yIntercept, p1x, p1y, p2x, p2y);
}
__device__ bool gpuIsDownSide(float px, float py, float slope, float yIntercept, float p1x, float p1y, float p2x, float p2y){
	return !gpuIsUpSide(px, py, slope, yIntercept, p1x, p1y, p2x, p2y);
}