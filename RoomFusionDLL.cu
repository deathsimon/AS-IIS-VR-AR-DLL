#include "stdio.h"
#include "RoomFusionDLL.cuh"

// 初始化CUDA，但其實這個函數也可以不呼叫，使用CUDA的預設值就OK
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

// Wrapper函數，呼叫執行GPU版本的深度套用
void runGPUApplyDepth(unsigned char* image, float* depth, int imageWidth, int imageHeight, float threshold){
	dim3 dimBlock(imageHeight, 1);
	dim3 dimGrid(imageWidth, 1);
	gpuApplyDepth << <dimGrid, dimBlock >> >(image, depth, imageWidth, imageHeight, threshold);
}
// Wrapper函數，呼叫執行GPU版本的深度校正(四邊形修正)
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
// 在GPU上執行深度套用，給予原始影像(image，4-channel)與深度資料(depth，1-channel)，把超過threshold的部分挖掉變成透明
__global__  void gpuApplyDepth(unsigned char* image, float* depth, int imageWidth, int imageHeight, float threshold){
	int w = blockIdx.x;
	int h = threadIdx.x;
	if (w < imageWidth && h < imageHeight){ // 須確保沒有超出範圍
		int positionIndex = h * imageWidth + w; // 由於image是一維陣列，需要自行計算某pixel的位置
		int pixelIndex = positionIndex * 4;  // 因為是4-channel，因此一次要跳4個byte才是一個pixel
		float depthVal = depth[positionIndex]; // 取出深度值
		if (depthVal > threshold){
			// 挖掉變成透明
			image[pixelIndex + 0] = 0;
			image[pixelIndex + 1] = 128; // 這個綠色區域是測試用的，在Unity中是看不到的
			image[pixelIndex + 2] = 0;
			image[pixelIndex + 3] = 0;
		}
	}
}
// 在GPU上進行深度修正，限制在四邊形之中
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
		int y = imageHeight - h - 1; // Y座標標需要顛倒，以滿足Y軸由下而上的規則
		if (
				// 分別在GPU上套用四個邊的修正
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

// 以下四個函數是用GPU檢查某一個點是不是在某一條線的右方/上方/左方/下方
// 參數：px、py：要檢查的點
//		 slope、yIntercept：該條線的斜率與Y截距
//       p1x, p1y, p2x, p2y：該條線經過的兩個點(p1與p2)的座標

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