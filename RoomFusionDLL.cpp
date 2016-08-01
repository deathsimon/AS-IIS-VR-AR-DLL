#include "stdafx.h"
#include "RoomFusionDLL.h"
#include "RoomFusionDLL.cuh"

#include "RoomFusionInternal.h"

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



void rf_init(){
	zed_init();
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
			//copyMatFromGPU2CPU(mat_image, mat_gpu_image);
			return TRUE;
		}
		else{
			return FALSE;
		}
	}
	return FALSE; 
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
	zed_destory();
}

void* rf_getCulledImagePtr()
{
	return mat_image.data;
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

