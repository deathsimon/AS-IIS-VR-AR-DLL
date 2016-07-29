#pragma once
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#define DEGREE_PER_RADIAN 57.2957795
#define nearlyEqual(a,b) ( a - b < 0.005f && a - b > -0.005f )

#include <iostream>
using namespace std;

static void zed_init();
static void zed_destory();
static void copyMatData(sl::zed::Mat& dst, const sl::zed::Mat& src);
static void applyDepthMat_cpu(sl::zed::Mat& image, const sl::zed::Mat& depth, float threshold = 2);
static bool checkAngle();
static void computeCorrectionMat_cpu(sl::zed::Mat& correction);
static void applyCorrectionMat_cpu(sl::zed::Mat& depth, const sl::zed::Mat& correction);

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
	void computeSlope(){
		slope = (p2.h - p1.h) / (p2.w - p1.w);
		yIntercept = p1.h - p1.w * slope;
		cout << "Slope:" << slope << "Inter:" << yIntercept << endl;
	}
	void setFromPoint(const PixelPosition& vp1, const PixelPosition& vp2){
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
	bool isRightSide(float px, float py){
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
	bool isLeftSide(float px, float py){
		return !isRightSide(px, py);
	}
	bool isUpSide(float px, float py){
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
	bool isDownSide(float px, float py){
		return !isUpSide(px, py);
	}
};