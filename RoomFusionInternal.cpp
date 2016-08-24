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


// �s��v�����򥻸�T
int imageSize; // �j�pbytes�A�򥻤W�OimageWidth * imageHeight * Channel��(�q�`�O4)
int imageWidth; // �e�סApixels
int imageHeight; // ���סApixels

sl::zed::Camera* zed; // ZED�۾����������
sl::zed::Mat mat_image[2]; // �ΨӦbCPU�W�x�s�Ⲵ����l�v���C�o���ܼƥu���bCUDA-D3D interop�S�ҥήɤ~�Ϊ���
sl::zed::Mat mat_gpu_image[2]; // �bGPU�W�s��Ⲵ����l�v��
sl::zed::Mat mat_gpu_depth[2]; // �bGPU�W�s��Ⲵ���`�׸�T


// �|��έץ�
bool apply_depth = true; // �O�_�i��`�׮M��
PixelPosition correction_point[4]; // �|��Ϊ��|���I�A�Ӧ�Unity�]�w
Line corretion_line[4]; // �|��Ϊ��|����A�ѤW�z�|���I��X��

Eigen::Matrix4f position; // ZED tracking����m
Eigen::Matrix4f positionT; // �P�W�A�u�O�i����m�᪺����
sl::zed::TRACKING_STATE track_state; // ����ZED��tracking���A

// �Ω�CUDA-D3D interop
ID3D11Texture2D* nativeTexture[2] = {NULL}; // ��ӶK�ϫ��СA��Unity�]�w
cudaGraphicsResource* cuda_img[2]; // CUDA interop�Ϊ�����

bool textureInit = false; // ����texture_init�O�_�����\����

// ���ݩж�����
// �H�U�o���ܼƥi��P�ɳQWorker Thread(Ū������Server�W���v��)�PMain Thread(��ܨ�Unity)Ū��
// �]���ݭn��Mutex�O�@
unsigned char* remoteRoomTextureBuffers[2][6]; // ���buffer(�䤤�@�Ӭ�shadow buffer) * ���ӭ�
int remoteRoomTextureBufferIndex; // ���ܷ�e���i��Buffer�O���@�ӡC�t�@��Buffer�N�Oshadow buffer
bool remoteRoomTextureBufferUpdated;  // ���ܦ��S�����s�����ݩж��v��
bool remoteRoomTextureBufferUsed; // ���ܷ�e��X�Ӫ����ݩж��v���O�_���QUnityŪ���L�F
pthread_t worker_thread; // Worker Thread���ܪ�
pthread_mutex_t remoteBufferMutex; // �ΨӫO�@�W�z�X���ܼƥΪ�Mutex

// �n���N���`��threshold�A���O����
float depthThreshold = 2.0f;

// ��ж������ӭ����e�P���s���ܼơA��K����s��
int remoteBoxDim[] = { 
						BOX_FRONT_W, BOX_FRONT_H,
						BOX_BACK_W, BOX_BACK_H, 
						BOX_LEFT_W, BOX_LEFT_H,
						BOX_RIGHT_W, BOX_RIGHT_H,
						BOX_TOP_W, BOX_TOP_H,
						BOX_DOWN_W, BOX_DOWN_H 
						};
// ��ж������ӭ����j�p�s���ܼơA��K����s��
int remoteBoxDataSize[] = {
	BOX_FRONT_W * BOX_FRONT_H * REMOTE_TEXTURE_CHANNELS,
	BOX_BACK_W * BOX_BACK_H * REMOTE_TEXTURE_CHANNELS,
	BOX_LEFT_W * BOX_LEFT_H * REMOTE_TEXTURE_CHANNELS,
	BOX_RIGHT_W * BOX_RIGHT_H * REMOTE_TEXTURE_CHANNELS,
	BOX_TOP_W *   BOX_TOP_H * REMOTE_TEXTURE_CHANNELS,
	BOX_DOWN_W * BOX_DOWN_H * REMOTE_TEXTURE_CHANNELS
};


// �H�U�O���ݩж��b�������ծɥΪ��ܼơA�{�b�w�Τ���
int max_remote_frames = 0;
unsigned char** testRemoteBuffers[6];
int current_remote_frames = 0;
int update_delay_count = 0;


// logger�����A�M�w�O�_����~�T����X��зǿ�X
// �w�]FALSE�N��N���~�T����X�������
int keepError2stdout = FALSE;

// �H�U�o�@�Ӹ򥭭��u�q��������ƬOCPU�������A�ثe�w�Τ���
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

// �H�U�O��L���������
// ������l��
// �D�n�ت��O�}�ҰO���ɨѼg�J
void internal_init(){
	if (!keepError2stdout){
		freopen("RoomFusion.log", "w", stdout);
	}
}
// ����cleanup�A�ثe�S���Ʊ��ݭn��
void internal_destroy(){
}
// �]�w�O�_�N���~�T���]�w��зǿ�X
void error2stdout(int value){
	keepError2stdout = value;
}
// ���ݩж���l��
void remoteRoom_init(){
	// ���ܼƪ�l��
	remoteRoomTextureBufferIndex = 0;
	remoteRoomTextureBufferUpdated = false;
	remoteRoomTextureBufferUsed = false;
	// ��l�ƻ��ݩж���Buffer
	for (int buf_id = 0; buf_id < 2; buf_id++){ // ���buffer
		for (int i = 0; i < 6; i++){ // ���ӭ�
			remoteRoomTextureBuffers[buf_id][i] = (unsigned char*)malloc(remoteBoxDim[i * 2 + 0] * remoteBoxDim[i * 2 + 1] * sizeof(unsigned char)* REMOTE_TEXTURE_CHANNELS);
		}
	}
	
#ifndef READ_REMOTE_FROM_NETWORK // �O�_�q����Server�WŪ�����ݩж��v��
	// ���ըϥΡAŪ���������R�A�v��
	fillTestRemoteData(10);
#else
	// �إ�worker thread�PMutex
	pthread_mutex_init(&remoteBufferMutex, NULL);
	pthread_create(&worker_thread, NULL, worker_updateRemote, NULL);
#endif
}
// ���ըϥΡAŪ���������R�A�v��
// count�G�R�A�v��frame��
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
// ���ݩж�cleanup
void remoteRoom_destroy(){
#ifndef READ_REMOTE_FROM_NETWORK
	// ������եΪ��R�A�v��
	if (max_remote_frames > 0){
		for (int i = 0; i < 6; i++){
			for (int j = 0; j < max_remote_frames; j++){
				free(testRemoteBuffers[i][j]);
			}
			free(testRemoteBuffers[i]);
		}
	}
#else
	// ����worker thread
	pthread_cancel(worker_thread);
	pthread_join(worker_thread, NULL);
	pthread_mutex_destroy(&remoteBufferMutex);
	
	
#endif
	// ������Buffer
	for (int buf_id = 0; buf_id < 2; buf_id++){
		for (int i = 0; i < 6; i++){
			free(remoteRoomTextureBuffers[buf_id][i]);
		}
	}
	
}
// Worker Thread�ϥΪ����
// �w���q����ServerŪ���v���ø����Y
void* worker_updateRemote(void*){
	// �]�w��thread�i�Q�H�ɨ���
	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	// Ū����ƪ��L�a�j��
	for (;;){
		// �ˬd�O�_�n�פThread
		pthread_testcancel();
		// ��X�U�@����shadow index
		int shadow_index = remoteRoomTextureBufferIndex* -1 + 1; // 0, 1 toggle
		// ���ձqsocket���o�v���é�Jshadow buffer
		// �ǤJshadow_index�ӧi���n��J���@��buffer
		if (socket_retrieve_image(shadow_index)){
			// ���ݩж��v���w�g���\��J��shadow buffer
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
		// �קKCPU�t���L���A���C���o�v�����W�v
		// ���ƭ����H�ۻ��ݩж��۾���FPS�վ�
		Sleep(25);
	}
	return NULL;
}

// ���ݩж���s
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
	// ���ըϥ�
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
// ��l��CUDA-D3D interop
void texture_init(){
	if (!textureInit){
		cout << "Init D3D Texture..." << endl;
		for (int eye = 0; eye < 2; eye++){
			if (nativeTexture[eye]){
				// �إ߬�����CUDA����ֶi��j�w
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

// ��l��ZED����
void zed_init(){
	zed = new sl::zed::Camera(sl::zed::HD720);
	sl::zed::InitParams params;
	params.mode = sl::zed::MODE::QUALITY; // �o�̥i�H�վ�e��A�v�T��FPS�C��L�ﶵ��PERFORMANCE�PMEDIUM
	params.unit = sl::zed::UNIT::METER;
	params.verbose = true;
	params.coordinate = sl::zed::COORDINATE_SYSTEM::LEFT_HANDED | sl::zed::COORDINATE_SYSTEM::APPLY_PATH;

	sl::zed::ERRCODE zederr = zed->init(params);
	// �����v���e����T
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
	// ���ǥΨӦs��CPU�v�����ܼƥ����n���allocate_cpu�Ӥ��t�O����Ŷ��CGPU�����ΡC
	for (int eye = 0; eye < 2; eye++){
		mat_image[eye].allocate_cpu(imageWidth, imageHeight, TEXTURE_CHANNELS, sl::zed::UCHAR);
	}
	

}
// ZED����cleanup
void zed_destory(){
	if (zed){
		delete zed;
		zed = nullptr;
		// ���allocate_cpu������ݭn���deallocate
		for (int eye = 0; eye < 2; eye++){
			mat_image[eye].deallocate();
		}
	}
}
// �bCPU�W�A��src���v����ƽƻs��dst
// ���|�O�n�קK���mat�@�ΦP�@�Ӽv�����
void copyMatData(sl::zed::Mat& dst, const sl::zed::Mat& src){
	memcpy(dst.data, src.data, imageSize);
}
// �bCPU�W�M�β`�׹ϡA�ثe�w���ϥ�
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
// �bCPU�W��`�׹ϮM�Υ|��έץ��A�ثe�w���ϥ�
// (part 1�G�p��ץ��᪺0�B1 map)
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
// �bCPU�W��`�׹ϮM�Υ|��έץ��A�ثe�w���ϥ�
// (part 2�G�M��0�B1 map)
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
// Wrapper��ơG�bGPU�W�M�β`�׹�
void applyDepthMat_gpu(sl::zed::Mat& image, sl::zed::Mat& depth){
	runGPUApplyDepth(image.data, (float*)depth.data, imageWidth, imageHeight, depthThreshold);
}
// Wrapper��ơG�bGPU�W�M�Υ|��έץ�
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
// �NGPU mat�W���v����ƽƻs��CPU mat
// CPU mat�������ƥ�allocate_cpu
void copyMatFromGPU2CPU(sl::zed::Mat& dst, const sl::zed::Mat& src){
	cudaMemcpy(dst.data, src.data, sizeof(unsigned char)* imageSize, cudaMemcpyDeviceToHost);
}