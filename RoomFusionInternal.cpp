#include "stdafx.h"
#include "RoomFusionDLL.h"
#include "RoomFusionDLL.cuh"
#include "RoomFusionInternal.h"
#include "RoomFusionSocket.h"
#include <cuda_d3d11_interop.h>
#include <iostream>
#include <cstdio>
#include <pthread.h>

#include <opencv2/opencv.hpp>

using namespace std;


// 存放影像的基本資訊
int imageSize; // 大小bytes，基本上是imageWidth * imageHeight * Channel數(通常是4)
int imageWidth; // 寬度，pixels
int imageHeight; // 高度，pixels

sl::zed::Camera* zed; // ZED相機的物件指標
sl::zed::Mat mat_image[2]; // 用來在CPU上儲存兩眼的原始影像。這個變數只有在CUDA-D3D interop沒啟用時才用的到
sl::zed::Mat mat_gpu_image[2]; // 在GPU上存放兩眼的原始影像
sl::zed::Mat mat_gpu_depth[2]; // 在GPU上存放兩眼的深度資訊


// 四邊形修正
bool apply_depth = true; // 是否進行深度套用
PixelPosition correction_point[4]; // 四邊形的四個點，來自Unity設定
Line corretion_line[4]; // 四邊形的四個邊，由上述四個點算出來

Eigen::Matrix4f position; // ZED tracking的位置
Eigen::Matrix4f positionT; // 同上，只是進行轉置後的版本
sl::zed::TRACKING_STATE track_state; // 紀錄ZED的tracking狀態

// 用於CUDA-D3D interop
ID3D11Texture2D* nativeTexture[2] = {NULL}; // 兩個貼圖指標，由Unity設定
cudaGraphicsResource* cuda_img[2]; // CUDA interop用的指標

bool textureInit = false; // 紀錄texture_init是否有成功執行

// 遠端房間相關
// 以下這些變數可能同時被Worker Thread(讀取遠端Server上的影像)與Main Thread(顯示到Unity)讀取
// 因此需要用Mutex保護
unsigned char* remoteRoomTextureBuffers[2][6]; // 兩個buffer(其中一個為shadow buffer) * 六個面
int remoteRoomTextureBufferIndex; // 指示當前的可用Buffer是哪一個。另一個Buffer就是shadow buffer
bool remoteRoomTextureBufferUpdated;  // 指示有沒有抓到新的遠端房間影像
bool remoteRoomTextureBufferUsed; // 指示當前抓出來的遠端房間影像是否有被Unity讀取過了
pthread_t worker_thread; // Worker Thread的變物
pthread_mutex_t remoteBufferMutex; // 用來保護上述幾個變數用的Mutex

// 要取代的深度threshold，單位是公尺
float depthThreshold = 2.0f;

// 把房間的六個面的寬與高存成變數，方便往後存取
int remoteBoxDim[] = { 
						BOX_FRONT_W, BOX_FRONT_H,
						BOX_BACK_W, BOX_BACK_H, 
						BOX_LEFT_W, BOX_LEFT_H,
						BOX_RIGHT_W, BOX_RIGHT_H,
						BOX_TOP_W, BOX_TOP_H,
						BOX_DOWN_W, BOX_DOWN_H 
						};
// 把房間的六個面的大小存成變數，方便往後存取
int remoteBoxDataSize[] = {
	BOX_FRONT_W * BOX_FRONT_H * REMOTE_TEXTURE_CHANNELS,
	BOX_BACK_W * BOX_BACK_H * REMOTE_TEXTURE_CHANNELS,
	BOX_LEFT_W * BOX_LEFT_H * REMOTE_TEXTURE_CHANNELS,
	BOX_RIGHT_W * BOX_RIGHT_H * REMOTE_TEXTURE_CHANNELS,
	BOX_TOP_W *   BOX_TOP_H * REMOTE_TEXTURE_CHANNELS,
	BOX_DOWN_W * BOX_DOWN_H * REMOTE_TEXTURE_CHANNELS
};


// 以下是遠端房間在本機測試時用的變數，現在已用不到
int max_remote_frames = 0;
unsigned char** testRemoteBuffers[6];
int current_remote_frames = 0;
int update_delay_count = 0;


// logger相關，決定是否把錯誤訊息輸出到標準輸出
// 預設FALSE代表將錯誤訊息輸出到紀錄檔
int keepError2stdout = FALSE;

// 以下這一個跟平面線段有關的函數是CPU版本的，目前已用不到
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

// 以下是其他的內部函數
// 內部初始化
// 主要目的是開啟記錄檔供寫入
void internal_init(){
	if (!keepError2stdout){
		freopen("RoomFusion.log", "w", stdout);
	}
}
// 內部cleanup，目前沒有事情需要做
void internal_destroy(){
}
// 設定是否將錯誤訊息設定到標準輸出
void error2stdout(int value){
	keepError2stdout = value;
}
// 遠端房間初始化
void remoteRoom_init(){
	// 基本變數初始化
	remoteRoomTextureBufferIndex = 0;
	remoteRoomTextureBufferUpdated = false;
	remoteRoomTextureBufferUsed = false;
	// 初始化遠端房間的Buffer
	for (int buf_id = 0; buf_id < 2; buf_id++){ // 兩個buffer
		for (int i = 0; i < 6; i++){ // 六個面
			remoteRoomTextureBuffers[buf_id][i] = (unsigned char*)malloc(remoteBoxDim[i * 2 + 0] * remoteBoxDim[i * 2 + 1] * sizeof(unsigned char)* REMOTE_TEXTURE_CHANNELS);
		}
	}
	
#ifndef READ_REMOTE_FROM_NETWORK // 是否從遠端Server上讀取遠端房間影像
	// 測試使用，讀取本機的靜態影像
	fillTestRemoteData(10);
#else
	// 建立worker thread與Mutex
	pthread_mutex_init(&remoteBufferMutex, NULL);
	pthread_create(&worker_thread, NULL, worker_updateRemote, NULL);
#endif
}
// 測試使用，讀取本機的靜態影像
// count：靜態影像frame數
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
// 遠端房間cleanup
void remoteRoom_destroy(){
#ifndef READ_REMOTE_FROM_NETWORK
	// 釋放測試用的靜態影像
	if (max_remote_frames > 0){
		for (int i = 0; i < 6; i++){
			for (int j = 0; j < max_remote_frames; j++){
				free(testRemoteBuffers[i][j]);
			}
			free(testRemoteBuffers[i]);
		}
	}
#else
	// 停止worker thread
	pthread_cancel(worker_thread);
	pthread_join(worker_thread, NULL);
	pthread_mutex_destroy(&remoteBufferMutex);
	
	
#endif
	// 釋放兩個Buffer
	for (int buf_id = 0; buf_id < 2; buf_id++){
		for (int i = 0; i < 6; i++){
			free(remoteRoomTextureBuffers[buf_id][i]);
		}
	}
	
}
// Worker Thread使用的函數
// 定期從遠端Server讀取影像並解壓縮
void* worker_updateRemote(void*){
	// 設定此thread可被隨時取消
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	// 讀取資料的無窮迴圈
	for (;;){
		// 檢查是否要終止此Thread
		pthread_testcancel();
		// 算出下一次的shadow index
		int shadow_index = remoteRoomTextureBufferIndex* -1 + 1; // 0, 1 toggle
		// 嘗試從socket取得影像並放入shadow buffer
		// 傳入shadow_index來告知要放入哪一個buffer
		if (socket_retrieve_image(shadow_index)){
			// 遠端房間影像已經成功放入到shadow buffer
			pthread_mutex_lock(&remoteBufferMutex);
			// if current buffer is used, swap buffer index
			if (remoteRoomTextureBufferUsed){
				remoteRoomTextureBufferIndex = shadow_index;
				remoteRoomTextureBufferUpdated = true;
			}
			else{
				// current buffer is not used, just untouch
			}
			pthread_mutex_unlock(&remoteBufferMutex);
		}
		// 避免CPU負載過高，降低取得影像的頻率
		// 此數值應隨著遠端房間相機的FPS調整
		Sleep(25);
	}
	return NULL;
}

// 遠端房間更新
bool remoteRoom_update(){
#ifdef READ_REMOTE_FROM_NETWORK
	// check if we have new data
	bool result;
	pthread_mutex_lock(&remoteBufferMutex);
	result = remoteRoomTextureBufferUpdated;
	if(result){
		remoteRoomTextureBufferUpdated = false; // mark as used
		remoteRoomTextureBufferUsed = false;
	}
	else{
		remoteRoomTextureBufferUsed = true;
	}
	pthread_mutex_unlock(&remoteBufferMutex);
	return result;

#else
	// 測試使用
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
// 初始化CUDA-D3D interop
void texture_init(){
	if (!textureInit){
		cout << "Init D3D Texture..." << endl;
		for (int eye = 0; eye < 2; eye++){
			if (nativeTexture[eye]){
				// 建立相關的CUDA物件併進行綁定
				cudaError_t err;
				err = cudaGraphicsD3D11RegisterResource(&cuda_img[eye], nativeTexture[eye], cudaGraphicsMapFlagsNone);
				if (err != cudaSuccess){
					cout << "Cannot create CUDA texture! " << cudaGetErrorString(err) << endl;
					return;
				}
				else{
					cout << "CUDA texture from D3D11 created" << endl;
				}
				cudaGraphicsMapResources(1, &cuda_img[eye], 0);
			}
			else{
				cout << "Cannot find native texture ptr :" << eye << endl;
				return;
			}
		}
		textureInit = true;
	}
}
// CUDA-D3D cleanup
void texture_destroy(){
	if (textureInit){
		for (int eye = 0; eye < 2; eye++){
			if (nativeTexture[eye]){
				cudaGraphicsUnmapResources(1, &cuda_img[eye], 0);
				nativeTexture[eye] = NULL;
			}
		}
		textureInit = false;
	}
}

// 初始化ZED物件
void zed_init(){
	zed = new sl::zed::Camera(sl::zed::HD720);
	sl::zed::InitParams params;
	params.mode = sl::zed::MODE::QUALITY; // 這裡可以調整畫質，影響到FPS。其他選項為PERFORMANCE與MEDIUM
	params.unit = sl::zed::UNIT::METER;
	params.verbose = true;
	params.coordinate = sl::zed::COORDINATE_SYSTEM::LEFT_HANDED | sl::zed::COORDINATE_SYSTEM::APPLY_PATH;

	sl::zed::ERRCODE zederr = zed->init(params);
	// 紀錄影像寬高資訊
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
	// 那些用來存放CPU影像的變數必須要手動allocate_cpu來分配記憶體空間。GPU的不用。
	for (int eye = 0; eye < 2; eye++){
		mat_image[eye].allocate_cpu(imageWidth, imageHeight, TEXTURE_CHANNELS, sl::zed::UCHAR);
	}
	

}
// ZED物件cleanup
void zed_destory(){
	if (zed){
		delete zed;
		zed = nullptr;
		// 手用allocate_cpu的物件需要手動deallocate
		for (int eye = 0; eye < 2; eye++){
			mat_image[eye].deallocate();
		}
	}
}
// 在CPU上，把src的影像資料複製到dst
// 此舉是要避免兩個mat共用同一個影像資料
void copyMatData(sl::zed::Mat& dst, const sl::zed::Mat& src){
	memcpy(dst.data, src.data, imageSize);
}
// 在CPU上套用深度圖，目前已不使用
void applyDepthMat_cpu(sl::zed::Mat& image, const sl::zed::Mat& depth){
	for (int h = 0; h < image.height; h++){
		for (int w = 0; w < image.width; w++){
			int index = h * image.width + w;
			float depthVal = ((float*)depth.data)[index];
			if (depthVal > depthThreshold){ // too depth
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
// 在CPU上對深度圖套用四邊形修正，目前已不使用
// (part 1：計算修正後的0、1 map)
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
// 在CPU上對深度圖套用四邊形修正，目前已不使用
// (part 2：套用0、1 map)
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
// Wrapper函數：在GPU上套用深度圖
void applyDepthMat_gpu(sl::zed::Mat& image, sl::zed::Mat& depth){
	runGPUApplyDepth(image.data, (float*)depth.data, imageWidth, imageHeight, depthThreshold);
}
// Wrapper函數：在GPU上套用四邊形修正
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
// 將GPU mat上的影像資料複製到CPU mat
// CPU mat必須先事先allocate_cpu
void copyMatFromGPU2CPU(sl::zed::Mat& dst, const sl::zed::Mat& src){
	cudaMemcpy(dst.data, src.data, sizeof(unsigned char)* imageSize, cudaMemcpyDeviceToHost);
}