#include <cuda_runtime.h>
bool InitCUDA();
void runGPUUpdate(unsigned char* img, int imageWidth, int imageHeight);
__global__ void gpuUpdateAlpha(unsigned char* img, int imageWidth, int imageHeight);