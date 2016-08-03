#pragma once
#ifdef ROOMFUSIONDLL_EXPORTS
#define ROOMFUSIONDLL_API __declspec(dllexport) 
#else
#define ROOMFUSIONDLL_API __declspec(dllimport) 
#endif

#define TRUE 1
#define FALSE 0

#define TEXTURE_CHANNELS 4
#define REMOTE_TEXTURE_CHANNELS 3

#define RECT_LT 0
#define RECT_RT 1
#define RECT_LD 2
#define RECT_RD 3

#define LINE_TOP 0
#define LINE_DOWN 1
#define LINE_LEFT 2
#define LINE_RIGHT 3

#define SIDE_FRONT 0
#define SIDE_BACK 1
#define SIDE_LEFT 2
#define SIDE_RIGHT 3
#define SIDE_TOP 4
#define SIDE_DOWN 5

#define BOX_FRONT_W 1416
#define BOX_FRONT_H 339

#define BOX_BACK_W 803
#define BOX_BACK_H 193

#define BOX_LEFT_W 451
#define BOX_LEFT_H 193

#define BOX_RIGHT_W 793
#define BOX_RIGHT_H 339

#define BOX_TOP_W 805
#define BOX_TOP_H 451

#define BOX_DOWN_W 803
#define BOX_DOWN_H 451


#define D3D_CUDA_INTEROP

extern "C" {
	
	ROOMFUSIONDLL_API void rf_init();
	ROOMFUSIONDLL_API void rf_destroy();
	ROOMFUSIONDLL_API int rf_update();

	// for observer room
	ROOMFUSIONDLL_API void rf_setD3D11TexturePtr(void*);

	ROOMFUSIONDLL_API void* rf_getCulledImagePtr();
	ROOMFUSIONDLL_API int rf_getImageSize();
	ROOMFUSIONDLL_API int rf_getImageWidth();
	ROOMFUSIONDLL_API int rf_getImageHeight();

	ROOMFUSIONDLL_API float* rf_getPositionPtr(); // return null if tracking unavailable 

	ROOMFUSIONDLL_API void rf_setApplyDepth(int);
	ROOMFUSIONDLL_API void rf_setCorrectionPixel(int position, float w, float h);
	ROOMFUSIONDLL_API void rf_computeCorrection();

	ROOMFUSIONDLL_API float rf_getZedFPS();


	// for remote room
	ROOMFUSIONDLL_API void* rf_getRemoteRoomTexturePtr(int side);

	
};
