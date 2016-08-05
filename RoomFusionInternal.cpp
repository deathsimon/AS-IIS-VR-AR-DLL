#include "stdafx.h"
#include "RoomFusionDLL.h"
#include "RoomFusionDLL.cuh"
#include "RoomFusionInternal.h"
#include "RoomFusionSocket.h"
#include <cuda_d3d11_interop.h>
#include <iostream>
#include <fstream>
#include <cstdio>

#include <opencv2/opencv.hpp>

using namespace std;


int imageSize;
int imageWidth;
int imageHeight;

sl::zed::Camera* zed;
sl::zed::Mat mat_image;
sl::zed::Mat mat_gpu_image;
sl::zed::Mat mat_gpu_depth;


// rotation fix
bool apply_depth = true;
PixelPosition correction_point[4];
Line corretion_line[4];

Eigen::Matrix4f position;
Eigen::Matrix4f positionT;
sl::zed::TRACKING_STATE track_state;

// D3D-cuda interop
ID3D11Texture2D* nativeTexture = NULL;
cudaGraphicsResource* cuda_img;

// remote room texture memory
unsigned char* remoteRoomTextureBuffers[6];
int remoteBoxDim[] = { 
						BOX_FRONT_W, BOX_FRONT_H,
						BOX_BACK_W, BOX_BACK_H, 
						BOX_LEFT_W, BOX_LEFT_H,
						BOX_RIGHT_W, BOX_RIGHT_H,
						BOX_TOP_W, BOX_TOP_H,
						BOX_DOWN_W, BOX_DOWN_H 
						};
int remoteBoxDataSize[] = {
	BOX_FRONT_W * BOX_FRONT_H * REMOTE_TEXTURE_CHANNELS,
	BOX_BACK_W * BOX_BACK_H * REMOTE_TEXTURE_CHANNELS,
	BOX_LEFT_W * BOX_LEFT_H * REMOTE_TEXTURE_CHANNELS,
	BOX_RIGHT_W * BOX_RIGHT_H * REMOTE_TEXTURE_CHANNELS,
	BOX_TOP_W *   BOX_TOP_H * REMOTE_TEXTURE_CHANNELS,
	BOX_DOWN_W * BOX_DOWN_H * REMOTE_TEXTURE_CHANNELS
};


// logger
ofstream fout;

// remote room test
int max_remote_frames = 0;
unsigned char** testRemoteBuffers[6];
int current_remote_frames = 0;
int update_delay_count = 0;


void Line::computeSlope(){
	slope = (p2.h - p1.h) / (p2.w - p1.w);
	yIntercept = p1.h - p1.w * slope;
}
void Line::setFromPoint(const PixelPosition& vp1, const PixelPosition& vp2){
	if (vp1.h > vp2.h){
		p1 = vp2;
		p2 = vp1;
	}
	else{
		p1 = vp1;
		p2 = vp2;
	}
	computeSlope();
}
bool Line::isRightSide(float px, float py){
	if (nearlyEqual(p2.h, p1.h)){ // horz
		return false;
	}
	else if (nearlyEqual(p2.w, p1.w)){ // vertical
		return px > p1.w;
	}
	float cSolution = (slope*px) + yIntercept;
	if (py > cSolution){
		return p2.w <= p1.w;
	}
	else{
		return p2.w > p1.w;
	}
}
bool Line::isLeftSide(float px, float py){
	return !isRightSide(px, py);
}
bool Line::isUpSide(float px, float py){
	if (nearlyEqual(p2.w - p1.w, 0)){ // vertical
		return false;
	}
	if (slope > 0){
		return isLeftSide(px, py);
	}
	else{
		return isRightSide(px, py);
	}
}
bool Line::isDownSide(float px, float py){
	return !isUpSide(px, py);
}

// normal functions



void internal_init(){
	fout.open("RoomFusion.log");
}

void internal_destroy(){
	fout.close();
}

void remoteRoom_init(){
	// box dimension
	
	// buffer
	for (int i = 0; i < 6; i++){
		remoteRoomTextureBuffers[i] = (unsigned char*)malloc(remoteBoxDim[i * 2 + 0] * remoteBoxDim[i * 2 + 1] * sizeof(unsigned char)* REMOTE_TEXTURE_CHANNELS);
	}
	// for testing propose: load static images
#ifndef READ_REMOTE_FROM_NETWORK
	fillTestRemoteData(10);
#endif
}

void fillTestRemoteData(int count){
	
	max_remote_frames = count;
	const char* name_prefix = "C:\\Users\\sinica-iis\\Desktop\\107\\";
	const char* name_suffix_map[] = {"5", "6", "1", "2", "3", "4"};
	char image_name[200];
	for (int i = 0; i < 6; i++){
		testRemoteBuffers[i] = (unsigned char**)malloc(max_remote_frames * sizeof(unsigned char*));
		for (int j = 0; j < max_remote_frames; j++){
			testRemoteBuffers[i][j] = (unsigned char*)malloc(remoteBoxDim[i * 2 + 0] * remoteBoxDim[i * 2 + 1] * sizeof(unsigned char)* REMOTE_TEXTURE_CHANNELS);
			// read image
			memset(image_name, 0, sizeof(image_name));
			sprintf_s(image_name, "%simage_%d_%s.png", name_prefix, j + 1, name_suffix_map[i]);
			cv::Mat img = cv::imread(image_name, cv::IMREAD_UNCHANGED);
			memcpy(testRemoteBuffers[i][j], img.data, img.size().area() * img.elemSize());
			img.deallocate();
		}
		memcpy(remoteRoomTextureBuffers[i], testRemoteBuffers[i][current_remote_frames], remoteBoxDim[i * 2 + 0] * remoteBoxDim[i * 2 + 1] * sizeof(unsigned char)* REMOTE_TEXTURE_CHANNELS);
		
	}
	
}

void remoteRoom_destroy(){
#ifndef READ_REMOTE_FROM_NETWORK
	// free test
	if (max_remote_frames > 0){
		for (int i = 0; i < 6; i++){
			for (int j = 0; j < max_remote_frames; j++){
				free(testRemoteBuffers[i][j]);
			}
			free(testRemoteBuffers[i]);
		}
	}
#endif
	// free normal
	free(remoteRoomTextureBuffers[SIDE_FRONT]);
	free(remoteRoomTextureBuffers[SIDE_BACK]);
	free(remoteRoomTextureBuffers[SIDE_LEFT]);
	free(remoteRoomTextureBuffers[SIDE_RIGHT]);
	free(remoteRoomTextureBuffers[SIDE_TOP]);
	free(remoteRoomTextureBuffers[SIDE_DOWN]);
}

bool remoteRoom_update(){
#ifdef READ_REMOTE_FROM_NETWORK
	return socket_retrieve_image();

#else
	if (max_remote_frames > 0){
		update_delay_count++;
		if (update_delay_count >= 2 ){
			update_delay_count = 0;
			current_remote_frames = (current_remote_frames + 1) % max_remote_frames;
			for (int i = 0; i < 6; i++){
				memcpy(remoteRoomTextureBuffers[i], testRemoteBuffers[i][current_remote_frames], remoteBoxDim[i * 2 + 0] * remoteBoxDim[i * 2 + 1] * sizeof(unsigned char)* REMOTE_TEXTURE_CHANNELS);
			}
		}
	}
	return true
#endif
}

void texture_init(){
	if (nativeTexture){
		fout << "Native Texture Ptr:" << nativeTexture << endl;
		cudaError_t err;
		err = cudaGraphicsD3D11RegisterResource(&cuda_img, nativeTexture, cudaGraphicsMapFlagsNone);
		if (err != cudaSuccess){
			fout << "Cannot create CUDA texture! " << cudaGetErrorString(err) << endl;
			nativeTexture = NULL;
			return;
		}
		else{
			fout << "CUDA texture from D3D11 created" << endl;
		}
		cudaGraphicsMapResources(1, &cuda_img, 0);
	}
	else{
		fout << "Cannot find native texture ptr" << endl;
		return;
	}
}

void texture_destroy(){
	if (nativeTexture){
		cudaGraphicsUnmapResources(1, &cuda_img, 0);
		nativeTexture = NULL;
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
	imageSize = imageHeight * imageWidth * TEXTURE_CHANNELS;
	if (zederr != sl::zed::SUCCESS)
	{
		cout << "ERROR: " << sl::zed::errcode2str(zederr) << endl;
		delete zed;
		zed = nullptr;
		return;
	}
	position.setIdentity(4, 4);
	zed->enableTracking(position, true);
	mat_image.allocate_cpu(imageWidth, imageHeight, TEXTURE_CHANNELS, sl::zed::UCHAR);

}
void zed_destory(){
	if (zed){
		delete zed;
		zed = nullptr;
		mat_image.deallocate();
	}
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

void applyDepthMat_gpu(sl::zed::Mat& image, sl::zed::Mat& depth, float threshold){
	runGPUApplyDepth(image.data, (float*)depth.data, imageWidth, imageHeight, threshold);
}

void applyCorrectionMat_gpu(sl::zed::Mat& depth,
	float left_slope, float left_inter, float left_p1x, float left_p1y, float left_p2x, float left_p2y,
	float right_slope, float right_inter, float right_p1x, float right_p1y, float right_p2x, float right_p2y,
	float top_slope, float top_inter, float top_p1x, float top_p1y, float top_p2x, float top_p2y,
	float down_slope, float down_inter, float down_p1x, float down_p1y, float down_p2x, float down_p2y
	){
	runGPUApplyCorrection((float*)depth.data, imageWidth, imageHeight, 
		left_slope, left_inter, left_p1x, left_p1y, left_p2x, left_p2y,
		right_slope, right_inter, right_p1x, right_p1y, right_p2x, right_p2y,
		top_slope, top_inter, top_p1x, top_p1y, top_p2x, top_p2y,
		down_slope, down_inter, down_p1x, down_p1y, down_p2x, down_p2y
		);
}

void copyMatFromGPU2CPU(sl::zed::Mat& dst, const sl::zed::Mat& src){
	cudaMemcpy(dst.data, src.data, sizeof(unsigned char)* imageSize, cudaMemcpyDeviceToHost);
}