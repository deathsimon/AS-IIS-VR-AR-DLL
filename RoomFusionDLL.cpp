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
extern sl::zed::Mat mat_image[2];
extern sl::zed::Mat mat_gpu_image[2];
extern sl::zed::Mat mat_gpu_depth[2];

extern bool apply_depth;
extern PixelPosition correction_point[4];
extern Line corretion_line[4];

extern Eigen::Matrix4f position;
extern Eigen::Matrix4f positionT;
extern sl::zed::TRACKING_STATE track_state;

extern ID3D11Texture2D* nativeTexture[2];
extern cudaGraphicsResource* cuda_img[2];

extern bool textureInit;

// remote room texture memory
extern unsigned char* remoteRoomTextureBuffers[2][6];
extern int remoteRoomTextureBufferIndex;
extern bool remoteRoomTextureBufferUpdated;

extern float depthThreshold;

void rf_init(){
	internal_init();
	zed_init();
	socket_init();
	remoteRoom_init();
}

void rf_setD3D11TexturePtr(int eye, void* ptr){
	nativeTexture[eye] = (ID3D11Texture2D*)ptr;
	cout << "Native Texture Ptr for eye " << eye << " :" << ptr << endl;
}
void rf_initD3DTexture(){
	texture_init();
}

void rf_error2stdout(int val){
	error2stdout(val);
}

int rf_update(){
	if (zed){
		if (!zed->grab(sl::zed::SENSING_MODE::FILL, true, true, false)){
			// tracking
			track_state = zed->getPosition(position, sl::zed::MAT_TRACKING_TYPE::PATH);
			// retrieve depth
			mat_gpu_depth[0] = mat_gpu_depth[1] = zed->retrieveMeasure_gpu(sl::zed::DEPTH);
			for (int eye = 0; eye < 2; eye++){
				// retrieve color image to gpu
				mat_gpu_image[eye] = zed->retrieveImage_gpu(eye == 0 ? sl::zed::SIDE::LEFT : sl::zed::SIDE::RIGHT);
				// apply
				if (apply_depth){

					// depth correction
					applyCorrectionMat_gpu(mat_gpu_depth[eye],
						corretion_line[LINE_LEFT].slope, corretion_line[LINE_LEFT].yIntercept, corretion_line[LINE_LEFT].p1.w, corretion_line[LINE_LEFT].p1.h, corretion_line[LINE_LEFT].p2.w, corretion_line[LINE_LEFT].p2.h,
						corretion_line[LINE_RIGHT].slope, corretion_line[LINE_RIGHT].yIntercept, corretion_line[LINE_RIGHT].p1.w, corretion_line[LINE_RIGHT].p1.h, corretion_line[LINE_RIGHT].p2.w, corretion_line[LINE_RIGHT].p2.h,
						corretion_line[LINE_TOP].slope, corretion_line[LINE_TOP].yIntercept, corretion_line[LINE_TOP].p1.w, corretion_line[LINE_TOP].p1.h, corretion_line[LINE_TOP].p2.w, corretion_line[LINE_TOP].p2.h,
						corretion_line[LINE_DOWN].slope, corretion_line[LINE_DOWN].yIntercept, corretion_line[LINE_DOWN].p1.w, corretion_line[LINE_DOWN].p1.h, corretion_line[LINE_DOWN].p2.w, corretion_line[LINE_DOWN].p2.h
						);
					// apply depth
					applyDepthMat_gpu(mat_gpu_image[eye], mat_gpu_depth[eye]);
				}
				// copy image

#ifdef D3D_CUDA_INTEROP
				// copy to D3D
				if (textureInit && nativeTexture[eye]){
					cudaArray_t arrIm;
					cudaGraphicsSubResourceGetMappedArray(&arrIm, cuda_img[eye], 0, 0);
					cudaMemcpy2DToArray(arrIm, 0, 0, mat_gpu_image[eye].data, mat_gpu_image[eye].step, imageWidth * 4, imageHeight, cudaMemcpyDeviceToDevice);
				}
#else
				// copy to cpu
				copyMatFromGPU2CPU(mat_image[eye], mat_gpu_image[eye]);
#endif
			}
			return TRUE;
		}
		else{
			return FALSE;
		}
	}
	return FALSE;
}

void rf_resetTracking(){
	zed->stopTracking();
	position.setIdentity(4, 4);
	zed->enableTracking(position, true);
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
	remoteRoom_destroy();
	socket_destroy();
	texture_destroy();
	zed_destory();
	internal_destroy();
}

void* rf_getRemoteRoomTexturePtr(int side){
	return remoteRoomTextureBuffers[remoteRoomTextureBufferIndex][side];
}

void* rf_getCulledImagePtr(int eye)
{
	return mat_image[eye].data;
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

float rf_getDepth(float w, float h){

	int x = int(w);
	int y = (imageHeight - (int)h - 1);
	cout << "Retrieve Depth at X:" << x << ", Y:" << y << endl;
	if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight){
		sl::zed::Mat mat_depth = zed->retrieveMeasure(sl::zed::DEPTH);
		return ((float*)mat_depth.data)[y* imageWidth + x];
	}
	else{
		return 0.0f;
	}
}

float rf_setDepthThreshold(float threshold){
	float ret = depthThreshold;
	depthThreshold = threshold;
	return ret;
}

int rf_isD3DInterop(){
#ifdef D3D_CUDA_INTEROP
	return TRUE;
#else
	return FALSE;
#endif
}

