#include "stdafx.h"
#include "RoomFusionDLL.h"
#include "RoomFusionDLL.cuh"

#include "RoomFusionInternal.h"
#include "RoomFusionSocket.h"

#include <iostream>


using namespace std;


// 以下全域變數來自RoomFusionInternal.cpp，請參閱該份檔案
extern int imageSize;
extern int imageWidth;
extern int imageHeight;

extern sl::zed::Camera* zed;
extern sl::zed::Mat mat_image[2];
extern sl::zed::Mat mat_gpu_image[2];
extern sl::zed::Mat mat_gpu_depth[2];

extern bool apply_depth;
extern PixelPosition correction_point[4];
extern Line corretion_line[4];

extern Eigen::Matrix4f position;
extern Eigen::Matrix4f positionT;
extern sl::zed::TRACKING_STATE track_state;

extern ID3D11Texture2D* nativeTexture[2];
extern cudaGraphicsResource* cuda_img[2];

extern bool textureInit;

// remote room texture memory
extern unsigned char* remoteRoomTextureBuffers[2][6];
extern int remoteRoomTextureBufferIndex;
extern bool remoteRoomTextureBufferUpdated;

extern float depthThreshold;

// 初始化RoomFusion相關的函數，注意順序是重要的。
// Unity啟動時應當先呼叫此函數，才可以進行其他的操作
void rf_init(){
	internal_init();
	zed_init();
	socket_init();
	remoteRoom_init();
}

// 存放Unity中的Texture Pointer，將之存放到內部的陣列中
// eye：左眼(0)或右眼(1)
// ptr：pointer，事實上的型態為 ID3D11Texture2D* 
void rf_setD3D11TexturePtr(int eye, void* ptr){
	nativeTexture[eye] = (ID3D11Texture2D*)ptr;
	cout << "Native Texture Ptr for eye " << eye << " :" << ptr << endl;
}
// 當兩眼的Texture pointer都設置好以後，呼叫此函數進行Texture初始化
void rf_initD3DTexture(){
	texture_init();
}

// 設定是否將錯誤輸出到標準輸出，或者是輸出到紀錄檔
// 這個功能主要是用在C++程式直接測試DLL的時候才會用到
// 一般來說Unity會呼叫此函數並傳入0，讓錯誤訊息不顯示到標準輸出，而是跑到紀錄檔中
void rf_error2stdout(int val){
	error2stdout(val);
}

// 更新函數
// 在Unity中必須要在每個Frame呼叫此函數來取得新的ZED影像
// 回傳值的TRUE/FALSE代表了這一次的update是否有取得新影像
int rf_update(){
	// 須先判定ZED指標是否存在，避免發生錯誤
	if (zed){
		// grab：請求ZED抓取下一個影像
		// 注意回傳值，成功時會回傳false
		// 第一個參數SENSING_MODE可以用FILL(填補模式)或STANDARD(一般模式)
		if (!zed->grab(sl::zed::SENSING_MODE::FILL, true, true, false)){
			// 更新ZED的Tracking功能，回傳值只是狀態，位置紀錄在position變數中
			track_state = zed->getPosition(position, sl::zed::MAT_TRACKING_TYPE::PATH);
			// 取得深度，這裡目前讓兩眼的深度用同一張圖
			mat_gpu_depth[0] = mat_gpu_depth[1] = zed->retrieveMeasure_gpu(sl::zed::DEPTH);
			for (int eye = 0; eye < 2; eye++){
				// 取得某一隻眼睛的影像，並存到GPU記憶體
				mat_gpu_image[eye] = zed->retrieveImage_gpu(eye == 0 ? sl::zed::SIDE::LEFT : sl::zed::SIDE::RIGHT);
				// 如果Unity中有請求套用深度(即apply_depth變數為true)，則開始進行深度取代
				if (apply_depth){
					// 進行角落偵測，把深度圖限制在四邊形中
					// 使用GPU運算
					applyCorrectionMat_gpu(mat_gpu_depth[eye],
						corretion_line[LINE_LEFT].slope, corretion_line[LINE_LEFT].yIntercept, corretion_line[LINE_LEFT].p1.w, corretion_line[LINE_LEFT].p1.h, corretion_line[LINE_LEFT].p2.w, corretion_line[LINE_LEFT].p2.h,
						corretion_line[LINE_RIGHT].slope, corretion_line[LINE_RIGHT].yIntercept, corretion_line[LINE_RIGHT].p1.w, corretion_line[LINE_RIGHT].p1.h, corretion_line[LINE_RIGHT].p2.w, corretion_line[LINE_RIGHT].p2.h,
						corretion_line[LINE_TOP].slope, corretion_line[LINE_TOP].yIntercept, corretion_line[LINE_TOP].p1.w, corretion_line[LINE_TOP].p1.h, corretion_line[LINE_TOP].p2.w, corretion_line[LINE_TOP].p2.h,
						corretion_line[LINE_DOWN].slope, corretion_line[LINE_DOWN].yIntercept, corretion_line[LINE_DOWN].p1.w, corretion_line[LINE_DOWN].p1.h, corretion_line[LINE_DOWN].p2.w, corretion_line[LINE_DOWN].p2.h
						);
					// 套用深度，把原始影像中深度太深的地方取代掉
					applyDepthMat_gpu(mat_gpu_image[eye], mat_gpu_depth[eye]);
				}
				// 影像複製
#ifdef D3D_CUDA_INTEROP
				// 在有啟動CUDA與DX11 interop的情況下，可以直接進行GPU-to-GPU記憶體複製
				// 檢查是否有進行過textrue的初始化
				if (textureInit && nativeTexture[eye]){
					// GPU記憶體複製
					cudaArray_t arrIm;
					cudaGraphicsSubResourceGetMappedArray(&arrIm, cuda_img[eye], 0, 0);
					cudaMemcpy2DToArray(arrIm, 0, 0, mat_gpu_image[eye].data, mat_gpu_image[eye].step, imageWidth * 4, imageHeight, cudaMemcpyDeviceToDevice);
				}
#else
				// 反之，如果沒有啟用，那麼就把GPU算出來的結果複製回CPU
				copyMatFromGPU2CPU(mat_image[eye], mat_gpu_image[eye]);
#endif
			}
			return TRUE;
		}
		else{
			return FALSE;
		}
	}
	return FALSE;
}

// 重製ZED的tracking功能，把當下的位置重新設為原點
// 主要用於校正
void rf_resetTracking(){
	zed->stopTracking(); // 關閉tracking
	position.setIdentity(4, 4); // 把目前位置設置為原點
	zed->enableTracking(position, true); // 再次啟動tracking
}

// 取得socket的延遲時間，包含傳輸與解壓縮
float rf_getSocketDelay(){
	return socket_getDelay();
}
// 更新遠端房間的內容(六面Box的貼圖)
int rf_updateRemoteRoom(){
	return remoteRoom_update();
}
// 取得影像大小(Bytes)
int rf_getImageSize(){
	return imageSize;
}
// 取得影像寬度(pixel)
int rf_getImageWidth(){
	return imageWidth;
}
// 取得影像高度(pixel)
int rf_getImageHeight(){
	return imageHeight;
}
// 終止前的cleanup處理，順序重要。基本上就是rf_init的顛倒
void rf_destroy(){
	remoteRoom_destroy();
	socket_destroy();
	texture_destroy();
	zed_destory();
	internal_destroy();
}
// 取得遠端房間用的記憶體指標
// 在Unity內會直接讀取這個指標的內容來刷新遠端房間的六個面的貼圖
// side：要取的面的代號
void* rf_getRemoteRoomTexturePtr(int side){
	return remoteRoomTextureBuffers[remoteRoomTextureBufferIndex][side];
}

// 用於沒有CUDA-D3D interop的情況
// 把算完的觀察者房間影像的pointer送回到Unity
void* rf_getCulledImagePtr(int eye)
{
	return mat_image[eye].data;
}

// 取得ZED的FPS
float rf_getZedFPS(){
	return zed->getCurrentFPS();
}
// 設定是否要套用深度
// result可以是0與非0的數字
// 必須要設定為非0的數字，才會把深度圖套用到原始影像上
void rf_setApplyDepth(int result){
	apply_depth = result;
}

// 設定四邊形的四個點
// position：RECT_LT、RECT_LD、RECT_RT、RECT_RD四個角落
void rf_setCorrectionPixel(int position, float w, float h){
	correction_point[position].set(w, h);
}

// 計算四邊行的四個邊，當四個點設定好之後，要呼叫這個函數來算出四個邊
void rf_computeCorrection(){
	corretion_line[LINE_TOP].setFromPoint(correction_point[RECT_LT], correction_point[RECT_RT]);
	corretion_line[LINE_DOWN].setFromPoint(correction_point[RECT_LD], correction_point[RECT_RD]);
	corretion_line[LINE_LEFT].setFromPoint(correction_point[RECT_LD], correction_point[RECT_LT]);
	corretion_line[LINE_RIGHT].setFromPoint(correction_point[RECT_RD], correction_point[RECT_RT]);
}

// 取得當前tracking位置的指標，讓Unity可以將之轉換成世界座標系
// 如果上次ZED有成功抓取到位置(TRACKING_GOOD)，則回傳位置資料的float指標
// 若否，則回傳NULL
float* rf_getPositionPtr(){
	if (track_state == sl::zed::TRACKING_GOOD){
		positionT = position.transpose(); // 進行轉置，主要是因為Unity格式與ZED格式不同
		return positionT.data();
	}
	else{
		return NULL;
	}
}

// 取得畫面中某一點的深度
// 參數是畫面上的二維座標
float rf_getDepth(float w, float h){

	int x = int(w);
	int y = (imageHeight - (int)h - 1); // 轉換Y座標，因為陣列是上而下儲存，而我們希望的座標系的Y軸是下而上
	cout << "Retrieve Depth at X:" << x << ", Y:" << y << endl;
	if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight){ // 確定座標範圍正確無誤
		sl::zed::Mat mat_depth = zed->retrieveMeasure(sl::zed::DEPTH); // 取得一張深度圖存到CPU
		return ((float*)mat_depth.data)[y* imageWidth + x]; // 回傳該點的深度資訊
	}
	else{
		return 0.0f;
	}
}

// 設定要取代的深度threshold，回傳原本的threshold
float rf_setDepthThreshold(float threshold){
	float ret = depthThreshold;
	depthThreshold = threshold;
	return ret;
}
// 檢察當下是否有啟用CUDA-D3D interop，主要讓Unity拿來判斷
int rf_isD3DInterop(){
#ifdef D3D_CUDA_INTEROP
	return TRUE;
#else
	return FALSE;
#endif
}

