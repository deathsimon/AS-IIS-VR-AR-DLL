#include "stdafx.h"
#include "RoomFusionDLL.h"
#include "RoomFusionDLL.cuh"
#include "RoomFusionInternal.h"


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

void zed_init(){
	zed = new sl::zed::Camera(sl::zed::HD720);
	sl::zed::InitParams params;
	params.mode = sl::zed::MODE::PERFORMANCE;
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
	mat_image.allocate_cpu(imageWidth, imageHeight, 4, sl::zed::UCHAR);

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