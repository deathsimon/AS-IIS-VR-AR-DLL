#pragma once
#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#define DEGREE_PER_RADIAN 57.2957795

static void zed_init();
static void zed_destory();
static void copyMatData(sl::zed::Mat& dst, const sl::zed::Mat& src);
static void applyDepthMat_cpu(sl::zed::Mat& image, const sl::zed::Mat& depth, float threshold = 2000);
static bool checkAngle();
static void computeCorrectionMat_cpu(sl::zed::Mat& correction);
static void applyCorrectionMat_cpu(sl::zed::Mat& depth, const sl::zed::Mat& correction);

struct PixelPosition{
	int w;
	int h;
	PixelPosition():w(0),h(0){}
	void set(int vw, int vh){
		w = vw;
		h = vh;
	}
};

struct Line{
	float cx;
	float cy;
	float d;
	bool slopeUp;
	Line() :cy(0), cx(0), d(0), slopeUp(false){}
	void computeSlopeUp(){
		slopeUp = cx * cy < 0;
	}
	void setFromCoef(float vx, float vy, float vd){
		cx = vx;
		cy = vy;
		d = vd;
		computeSlopeUp();
	}
	void setFromPoint(float x1, float y1, float x2, float y2){
		cx = y1 - y2;
		cy = x2 - x1;
		d = x1 * y2 - x2 * y1;
		computeSlopeUp();
	}
	void setFromPoint(const PixelPosition& p1, const PixelPosition& p2){
		setFromPoint(p1.w, p1.h, p2.w, p2.h);
	}
	bool isRightSide(float px, float py){
		return cx * px + cy * py + d > 0;
	}
	bool isLeftSide(float px, float py){
		return cx * px + cy * py + d < 0;
	}
	bool isUpSide(float px, float py){
		if (slopeUp){
			return isLeftSide(px, py);
		}
		else{
			return isRightSide(px, py);
		}
	}
	bool isDownSide(float px, float py){
		if (slopeUp){
			return isRightSide(px, py);
		}
		else{
			return isLeftSide(px, py);
		}
	}
};