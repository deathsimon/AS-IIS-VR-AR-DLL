// RoomFusionDLL.cpp : 定義 DLL 應用程式的匯出函式。
//

#include "stdafx.h"
#include "RoomFusionDLL.h"
#include "RoomFusionDLL.cuh"

#include "RoomFusionInternal.h"

#include <iostream>


using namespace std;

static int imageSize;
static int imageWidth;
static int imageHeight;
static unsigned char* gpudata;
// static unsigned char* image;

static sl::zed::Camera* zed;

static sl::zed::Mat mat_image;
static sl::zed::Mat mat_depth;
static sl::zed::Mat mat_correction;


// rotation fix
static bool apply_depth = true;
static PixelPosition correction_point[4];
static Line corretion_line[4];

static Eigen::Matrix4f position;
static Eigen::Matrix4f positionT;
static sl::zed::TRACKING_STATE track_state;


void rf_init(){
	// Init
	zed_init();

	// For testing propose: use Static picture
	/*
	cv::Mat img = cv::imread("C:\\Users\\sinica-iis\\Desktop\\input.png", cv::IMREAD_UNCHANGED);
	cv::Size s = img.size();

	imageHeight = s.height;
	imageWidth = s.width;
	imageSize = imageHeight * imageWidth * img.elemSize();

	image = new unsigned char[imageSize];
	memcpy(image, img.data, imageSize);
	InitCUDA();
	cudaMalloc((void**)&gpudata, sizeof(unsigned char)* imageSize);
	cudaMemcpy(gpudata, image, sizeof(unsigned char)* imageSize, cudaMemcpyHostToDevice);
	*/
}

int rf_update(){

	if (zed){
		if (!zed->grab(sl::zed::SENSING_MODE::FILL, true, true, false)){
			// tracking
			track_state = zed->getPosition(position, sl::zed::MAT_TRACKING_TYPE::PATH);
			// retrieve depth
			copyMatData(mat_depth, zed->retrieveMeasure(sl::zed::DEPTH));
			// retrieve color image
			mat_image = zed->retrieveImage(sl::zed::SIDE::LEFT);
			// apply
			if (apply_depth){
				computeCorrectionMat_cpu(mat_correction);
				applyCorrectionMat_cpu(mat_depth, mat_correction);
				applyDepthMat_cpu(mat_image, mat_depth);
			}
			return TRUE;
		}
		else{
			return FALSE;
		}
	}
	return FALSE; 
	/*
	runGPUUpdate(gpudata, imageWidth, imageHeight);
	cudaError err = cudaGetLastError();
	if (cudaSuccess != err)
	{
		fprintf(stderr, "cudaCheckError() failed: %s\n", cudaGetErrorString(err));
	}
	cudaMemcpy(image, gpudata, sizeof(unsigned char)* imageSize, cudaMemcpyDeviceToHost);
	*/
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
	//cudaFree(gpudata);
	//delete[] image;
}

void* rf_getCulledImagePtr()
{
	//return image;
	return mat_image.data;
}

void rf_setApplyDepth(int result){
	apply_depth = result;
}

void rf_setCorrectionPixel(int position, float w, float h){
	correction_point[position].set(w, h);
}

void copyMatData(sl::zed::Mat& dst, const sl::zed::Mat& src){
	memcpy(dst.data, src.data, imageSize);
}

void applyDepthMat_cpu(sl::zed::Mat& image, const sl::zed::Mat& depth, float threshold){
	for (int h = 0; h < image.height; h++){
		for (int w = 0; w < image.width; w++){
			int index = h * image.width + w;
			float depthVal = ((float*)depth.data)[index];
			if (depthVal > threshold){ // too depth
				// change this pixel to invisible
				int matIndex = index * image.channels;
				image.data[matIndex + 0] = 0;
				image.data[matIndex + 1] = 128;
				image.data[matIndex + 2] = 0;
				image.data[matIndex + 3] = 0;
			}
		}
	}
}

void computeCorrectionMat_cpu(sl::zed::Mat& correction){
	for (int h = 0; h < correction.height; h++){
		for (int w = 0; w < correction.width; w++){
			int index = h * correction.width + w;
			int y = correction.height - h - 1;
			if (corretion_line[LINE_TOP].isDownSide(w, y) &&
				corretion_line[LINE_DOWN].isUpSide(w, y) &&
				corretion_line[LINE_LEFT].isRightSide(w, y) &&
				corretion_line[LINE_RIGHT].isLeftSide(w, y)
				){
				correction.data[index] = 0; // transparent
			}
			else{
				correction.data[index] = 1; // keep
			}
		}
	}
}

void applyCorrectionMat_cpu(sl::zed::Mat& depth, const sl::zed::Mat& correction){
	for (int h = 0; h < depth.height; h++){
		for (int w = 0; w < depth.width; w++){
			int index = h * depth.width + w;
			if (correction.data[index] == 1){ // keep
				((float*)depth.data)[index] = 0.0f;
			}
		}
	}
}

void rf_computeCorrection(){
	// create the line
	corretion_line[LINE_TOP].setFromPoint(correction_point[RECT_LT], correction_point[RECT_RT]);
	corretion_line[LINE_DOWN].setFromPoint(correction_point[RECT_LD], correction_point[RECT_RD]);
	corretion_line[LINE_LEFT].setFromPoint(correction_point[RECT_LD], correction_point[RECT_LT]);
	corretion_line[LINE_RIGHT].setFromPoint(correction_point[RECT_RD], correction_point[RECT_RT]);
	//cout << "Line TOP: cx = " << corretion_line[LINE_TOP].cx << ", cy = " << corretion_line[LINE_TOP].cy << ", d = " << corretion_line[LINE_TOP].d << ", is SlopeUp:" << endl;
	//cout << "Line DOWN: cx = " << corretion_line[LINE_DOWN].cx << ", cy = " << corretion_line[LINE_DOWN].cy << ", d = " << corretion_line[LINE_DOWN].d << ", is SlopeUp:" << endl;
	//cout << "Line LEFT: cx = " << corretion_line[LINE_LEFT].cx << ", cy = " << corretion_line[LINE_LEFT].cy << ", d = " << corretion_line[LINE_LEFT].d << ", is SlopeUp:" << endl;
	//cout << "Line RIGHT: cx = " << corretion_line[LINE_RIGHT].cx << ", cy = " << corretion_line[LINE_RIGHT].cy << ", d = " << corretion_line[LINE_RIGHT].d << ", is SlopeUp:" << endl;

}


float* rf_getPositionPtr(){
	// tracking
	if (track_state == sl::zed::TRACKING_GOOD){
		//cout << "Pose: \n" << position << endl;
		positionT = position.transpose();
		return positionT.data();
	}
	else{
		return NULL;
		//cout << "Tracking error" << endl;
	}
}

void zed_init(){
	zed = new sl::zed::Camera(sl::zed::HD720);
	sl::zed::InitParams params;
	params.mode = sl::zed::MODE::QUALITY;
	params.unit = sl::zed::UNIT::METER;
	params.verbose = true;
	params.coordinate = sl::zed::COORDINATE_SYSTEM::LEFT_HANDED | sl::zed::COORDINATE_SYSTEM::APPLY_PATH;
	
	sl::zed::ERRCODE zederr = zed->init(params);
	imageWidth = zed->getImageSize().width;
	imageHeight = zed->getImageSize().height;
	imageSize = imageHeight * imageWidth * 4;
	if (zederr != sl::zed::SUCCESS)
	{
		cout << "ERROR: " << sl::zed::errcode2str(zederr) << endl;
		delete zed;
		zed = nullptr;
		return;
	}
	position.setIdentity(4, 4);
	zed->enableTracking(position, true);
	mat_depth.allocate_cpu(imageWidth, imageHeight, 1, sl::zed::FLOAT);
	mat_correction.allocate_cpu(imageWidth, imageHeight, 1, sl::zed::UCHAR);

}
void zed_destory(){
	if (zed){
		delete zed;
		zed = nullptr;
		mat_depth.deallocate();
		mat_correction.deallocate();
	}
}


