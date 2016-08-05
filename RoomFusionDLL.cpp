#include "stdafx.h"
#include "RoomFusionDLL.h"
#include "RoomFusionDLL.cuh"

#include "RoomFusionInternal.h"
#include "RoomFusionSocket.h"

#include <iostream>


using namespace std;

extern int imageSize;
extern int imageWidth;
extern int imageHeight;

extern sl::zed::Camera* zed;
extern sl::zed::Mat mat_image;
extern sl::zed::Mat mat_gpu_image;
extern sl::zed::Mat mat_gpu_depth;

extern bool apply_depth;
extern PixelPosition correction_point[4];
extern Line corretion_line[4];

extern Eigen::Matrix4f position;
extern Eigen::Matrix4f positionT;
extern sl::zed::TRACKING_STATE track_state;

extern ID3D11Texture2D* nativeTexture;
extern cudaGraphicsResource* cuda_img;

// remote room texture memory
extern unsigned char* remoteRoomTextureBuffers[6];



extern ofstream fout;

void rf_init(){
	internal_init();
	zed_init();
	remoteRoom_init();
	socket_init();
}

void rf_setD3D11TexturePtr(void* ptr){
	nativeTexture = (ID3D11Texture2D*)ptr;
	texture_init();
}

int rf_update(){
	if (zed){
		if (!zed->grab(sl::zed::SENSING_MODE::FILL, true, true, false)){
			// tracking
			track_state = zed->getPosition(position, sl::zed::MAT_TRACKING_TYPE::PATH);
			// retrieve depth
			mat_gpu_depth = zed->retrieveMeasure_gpu(sl::zed::DEPTH);
			// retrieve color image to gpu
			mat_gpu_image = zed->retrieveImage_gpu(sl::zed::SIDE::LEFT);
			// apply
			if (apply_depth){
				// depth correction
				applyCorrectionMat_gpu(mat_gpu_depth, 
					corretion_line[LINE_LEFT].slope, corretion_line[LINE_LEFT].yIntercept, corretion_line[LINE_LEFT].p1.w, corretion_line[LINE_LEFT].p1.h, corretion_line[LINE_LEFT].p2.w, corretion_line[LINE_LEFT].p2.h,
					corretion_line[LINE_RIGHT].slope, corretion_line[LINE_RIGHT].yIntercept, corretion_line[LINE_RIGHT].p1.w, corretion_line[LINE_RIGHT].p1.h, corretion_line[LINE_RIGHT].p2.w, corretion_line[LINE_RIGHT].p2.h,
					corretion_line[LINE_TOP].slope, corretion_line[LINE_TOP].yIntercept, corretion_line[LINE_TOP].p1.w, corretion_line[LINE_TOP].p1.h, corretion_line[LINE_TOP].p2.w, corretion_line[LINE_TOP].p2.h,
					corretion_line[LINE_DOWN].slope, corretion_line[LINE_DOWN].yIntercept, corretion_line[LINE_DOWN].p1.w, corretion_line[LINE_DOWN].p1.h, corretion_line[LINE_DOWN].p2.w, corretion_line[LINE_DOWN].p2.h
					);
				// apply depth
				applyDepthMat_gpu(mat_gpu_image, mat_gpu_depth);
			}
			// copy image
#ifdef D3D_CUDA_INTEROP
			// copy to D3D
			if (nativeTexture){
				cudaArray_t arrIm;
				cudaGraphicsSubResourceGetMappedArray(&arrIm, cuda_img, 0, 0);
				cudaMemcpy2DToArray(arrIm, 0, 0, mat_gpu_image.data, mat_gpu_image.step, imageWidth * 4, imageHeight, cudaMemcpyDeviceToDevice);
			}
#else
			// copy to cpu
			copyMatFromGPU2CPU(mat_image, mat_gpu_image);
#endif
			return TRUE;
		}
		else{
			return FALSE;
		}
	}
	return FALSE; 
}

float rf_getSocketDelay(){
	return socket_getDelay();
}
int rf_updateRemoteRoom(){
	return remoteRoom_update();
}

int rf_getImageSize(){
	return imageSize;
}
int rf_getImageWidth(){
	return imageWidth;
}
int rf_getImageHeight(){
	return imageHeight;
}

void rf_destroy(){
	socket_destroy();
	remoteRoom_destroy();
	texture_destroy();
	zed_destory();
	internal_destroy();
}

void* rf_getRemoteRoomTexturePtr(int side){
	return remoteRoomTextureBuffers[side];
}

void* rf_getCulledImagePtr()
{
	return mat_image.data;
}

float rf_getZedFPS(){
	return zed->getCurrentFPS();
}

void rf_setApplyDepth(int result){
	apply_depth = result;
}

void rf_setCorrectionPixel(int position, float w, float h){
	correction_point[position].set(w, h);
}


void rf_computeCorrection(){
	corretion_line[LINE_TOP].setFromPoint(correction_point[RECT_LT], correction_point[RECT_RT]);
	corretion_line[LINE_DOWN].setFromPoint(correction_point[RECT_LD], correction_point[RECT_RD]);
	corretion_line[LINE_LEFT].setFromPoint(correction_point[RECT_LD], correction_point[RECT_LT]);
	corretion_line[LINE_RIGHT].setFromPoint(correction_point[RECT_RD], correction_point[RECT_RT]);
}


float* rf_getPositionPtr(){
	if (track_state == sl::zed::TRACKING_GOOD){
		positionT = position.transpose();
		return positionT.data();
	}
	else{
		return NULL;
	}
}

