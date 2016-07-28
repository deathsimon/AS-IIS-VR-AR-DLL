#pragma once
#ifdef ROOMFUSIONDLL_EXPORTS
#define ROOMFUSIONDLL_API __declspec(dllexport) 
#else
#define ROOMFUSIONDLL_API __declspec(dllimport) 
#endif

#define TRUE 1
#define FALSE 0

#define RECT_LT 0
#define RECT_RT 1
#define RECT_LD 2
#define RECT_RD 3

#define LINE_TOP 0
#define LINE_DOWN 1
#define LINE_LEFT 2
#define LINE_RIGHT 3

extern "C" {
	ROOMFUSIONDLL_API void rf_init();
	ROOMFUSIONDLL_API void rf_destroy();
	ROOMFUSIONDLL_API int rf_update();

	ROOMFUSIONDLL_API void* rf_getCulledImagePtr();
	ROOMFUSIONDLL_API int rf_getImageSize();
	ROOMFUSIONDLL_API int rf_getImageWidth();
	ROOMFUSIONDLL_API int rf_getImageHeight();

	ROOMFUSIONDLL_API void rf_setApplyDepth(int);
	ROOMFUSIONDLL_API void rf_setCorrectionPixel(int position, float w, float h);
	ROOMFUSIONDLL_API void rf_computeCorrection();
};
