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

void runGPUUpdate(unsigned char* img, int imageWidth, int imageHeight){
	//dim3 dimBlock(imageWidth, imageHeight);
	dim3 dimBlock(1024, 1);
	dim3 dimGrid(768, 1);
	gpuUpdateAlpha << <dimGrid, dimBlock >> >(img, imageWidth, imageHeight);
}

__global__ void gpuUpdateAlpha(unsigned char* img, int imageWidth, int imageHeight)
{
	int w = threadIdx.x;
	int h = blockIdx.x;
	if (w < imageWidth && h < imageHeight){
		int index = (h * imageWidth + w) * 4 + 3;
		unsigned char val = img[index];
		if (val >= 255){
			val = 1;
		}
		else if (val > 0)
		{
			++val;
		}
		img[index] = val;
	}
}