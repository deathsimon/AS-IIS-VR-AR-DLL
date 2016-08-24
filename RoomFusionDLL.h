#pragma once
#ifdef ROOMFUSIONDLL_EXPORTS
#define ROOMFUSIONDLL_API __declspec(dllexport) 
#else
#define ROOMFUSIONDLL_API __declspec(dllimport) 
#endif

// 布林值
#define TRUE 1
#define FALSE 0

// 眼睛
#define EYE_LEFT 0
#define EYE_RIGHT 1

// 貼圖的channel數，基本上都是BGRA 4-channel
#define TEXTURE_CHANNELS 4
#define REMOTE_TEXTURE_CHANNELS 4

// 定義四邊形的四個角落
#define RECT_LT 0
#define RECT_RT 1
#define RECT_LD 2
#define RECT_RD 3

// 定義四邊形的四個邊
#define LINE_TOP 0
#define LINE_DOWN 1
#define LINE_LEFT 2
#define LINE_RIGHT 3

// 遠端房間六面體的六個面
#define SIDE_FRONT 0
#define SIDE_BACK 1
#define SIDE_LEFT 2
#define SIDE_RIGHT 3
#define SIDE_TOP 4
#define SIDE_DOWN 5

// 遠端房間每一面的解析度(Pixel)
#define BOX_FRONT_W 1024
#define BOX_FRONT_H 259

#define BOX_BACK_W 1024
#define BOX_BACK_H 259

#define BOX_LEFT_W 826
#define BOX_LEFT_H 259

#define BOX_RIGHT_W 826
#define BOX_RIGHT_H 259

#define BOX_TOP_W 1024
#define BOX_TOP_H 826

#define BOX_DOWN_W 1024
#define BOX_DOWN_H 826

// 是否啟用CUDA-D3D interop
// 若不啟用，移除掉下面這一行即可
#define D3D_CUDA_INTEROP
 
// 函數匯出
// 詳細的函數說明請參與RoomFusionDLL.cpp
extern "C" {
	
	ROOMFUSIONDLL_API void rf_init();
	ROOMFUSIONDLL_API void rf_destroy();
	ROOMFUSIONDLL_API int rf_update();

	ROOMFUSIONDLL_API void rf_error2stdout(int);

	// for observer room
	ROOMFUSIONDLL_API void rf_setD3D11TexturePtr(int eye, void*);
	ROOMFUSIONDLL_API void rf_initD3DTexture();

	ROOMFUSIONDLL_API void* rf_getCulledImagePtr(int eye);
	ROOMFUSIONDLL_API int rf_getImageSize();
	ROOMFUSIONDLL_API int rf_getImageWidth();
	ROOMFUSIONDLL_API int rf_getImageHeight();

	ROOMFUSIONDLL_API float* rf_getPositionPtr(); // return null if tracking unavailable 

	ROOMFUSIONDLL_API void rf_setApplyDepth(int);
	ROOMFUSIONDLL_API float rf_setDepthThreshold(float threshold); // return old threshold

	ROOMFUSIONDLL_API void rf_setCorrectionPixel(int position, float w, float h);
	ROOMFUSIONDLL_API void rf_computeCorrection();
	ROOMFUSIONDLL_API float rf_getDepth(float w, float h);

	ROOMFUSIONDLL_API void rf_resetTracking();


	ROOMFUSIONDLL_API float rf_getZedFPS();



	// for remote room
	ROOMFUSIONDLL_API float rf_getSocketDelay();
	ROOMFUSIONDLL_API int rf_updateRemoteRoom();
	ROOMFUSIONDLL_API void* rf_getRemoteRoomTexturePtr(int side);

	// others
	ROOMFUSIONDLL_API int rf_isD3DInterop();

	
};
