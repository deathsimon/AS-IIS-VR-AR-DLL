#pragma once
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>
#include <D3D11.h>

#define DEGREE_PER_RADIAN 57.2957795
#define nearlyEqual(a,b) ( a - b < 0.005f && a - b > -0.005f )

#define READ_REMOTE_FROM_NETWORK

#include <iostream>
using namespace std;

void* worker_updateRemote(void*);
void remoteRoom_init();
void remoteRoom_destroy();
bool remoteRoom_update();
void internal_init();
void internal_destroy();
void texture_init();
void texture_destroy();
void zed_init();
void zed_destory();
void copyMatData(sl::zed::Mat& dst, const sl::zed::Mat& src);
void applyDepthMat_cpu(sl::zed::Mat& image, const sl::zed::Mat& depth, float threshold = 2);
void applyDepthMat_gpu(sl::zed::Mat& image, sl::zed::Mat& depth, float threshold = 2);
bool checkAngle();
void computeCorrectionMat_cpu(sl::zed::Mat& correction);
void applyCorrectionMat_cpu(sl::zed::Mat& depth, const sl::zed::Mat& correction);

void applyCorrectionMat_gpu(sl::zed::Mat& depth,
							float left_slope, float left_inter, float left_p1x, float left_p1y, float left_p2x, float left_p2y,
							float right_slope, float right_inter, float right_p1x, float right_p1y, float right_p2x, float right_p2y,
							float top_slope, float top_inter, float top_p1x, float top_p1y, float top_p2x, float top_p2y,
							float down_slope, float down_inter, float down_p1x, float down_p1y, float down_p2x, float down_p2y
							);
void copyMatFromGPU2CPU(sl::zed::Mat& dst, const sl::zed::Mat& src);

// Internal test
void fillTestRemoteData(int count);

// --


struct PixelPosition{
	float w;
	float h;
	PixelPosition():w(0),h(0){}
	void set(float vw, float vh){
		w = vw;
		h = vh;
	}
};

struct Line{
	PixelPosition p1, p2;
	float slope;
	float yIntercept;
	void computeSlope();
	void setFromPoint(const PixelPosition& vp1, const PixelPosition& vp2);
	bool isRightSide(float px, float py);
	bool isLeftSide(float px, float py);
	bool isUpSide(float px, float py);
	bool isDownSide(float px, float py);
};