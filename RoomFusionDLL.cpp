#include "stdafx.h"
#include "RoomFusionDLL.h"
#include "RoomFusionDLL.cuh"

#include "RoomFusionInternal.h"
#include "RoomFusionSocket.h"

#include <iostream>


using namespace std;


// �H�U�����ܼƨӦ�RoomFusionInternal.cpp�A�аѾ\�ӥ��ɮ�
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

// ��l��RoomFusion��������ơA�`�N���ǬO���n���C
// Unity�Ұʮ�������I�s����ơA�~�i�H�i���L���ާ@
void rf_init(){
	internal_init();
	zed_init();
	socket_init();
	remoteRoom_init();
}

// �s��Unity����Texture Pointer�A�N���s��줺�����}�C��
// eye�G����(0)�Υk��(1)
// ptr�Gpointer�A�ƹ�W�����A�� ID3D11Texture2D* 
void rf_setD3D11TexturePtr(int eye, void* ptr){
	nativeTexture[eye] = (ID3D11Texture2D*)ptr;
	cout << "Native Texture Ptr for eye " << eye << " :" << ptr << endl;
}
// ��Ⲵ��Texture pointer���]�m�n�H��A�I�s����ƶi��Texture��l��
void rf_initD3DTexture(){
	texture_init();
}

// �]�w�O�_�N���~��X��зǿ�X�A�Ϊ̬O��X�������
// �o�ӥ\��D�n�O�ΦbC++�{����������DLL���ɭԤ~�|�Ψ�
// �@��ӻ�Unity�|�I�s����ƨöǤJ0�A�����~�T������ܨ�зǿ�X�A�ӬO�]������ɤ�
void rf_error2stdout(int val){
	error2stdout(val);
}

// ��s���
// �bUnity�������n�b�C��Frame�I�s����ƨӨ��o�s��ZED�v��
// �^�ǭȪ�TRUE/FALSE�N��F�o�@����update�O�_�����o�s�v��
int rf_update(){
	// �����P�wZED���ЬO�_�s�b�A�קK�o�Ϳ��~
	if (zed){
		// grab�G�ШDZED����U�@�Ӽv��
		// �`�N�^�ǭȡA���\�ɷ|�^��false
		// �Ĥ@�ӰѼ�SENSING_MODE�i�H��FILL(��ɼҦ�)��STANDARD(�@��Ҧ�)
		if (!zed->grab(sl::zed::SENSING_MODE::FILL, true, true, false)){
			// ��sZED��Tracking�\��A�^�ǭȥu�O���A�A��m�����bposition�ܼƤ�
			track_state = zed->getPosition(position, sl::zed::MAT_TRACKING_TYPE::PATH);
			// ���o�`�סA�o�̥ثe���Ⲵ���`�ץΦP�@�i��
			mat_gpu_depth[0] = mat_gpu_depth[1] = zed->retrieveMeasure_gpu(sl::zed::DEPTH);
			for (int eye = 0; eye < 2; eye++){
				// ���o�Y�@���������v���A�æs��GPU�O����
				mat_gpu_image[eye] = zed->retrieveImage_gpu(eye == 0 ? sl::zed::SIDE::LEFT : sl::zed::SIDE::RIGHT);
				// �p�GUnity�����ШD�M�β`��(�Yapply_depth�ܼƬ�true)�A�h�}�l�i��`�ר��N
				if (apply_depth){
					// �i�樤�������A��`�׹ϭ���b�|��Τ�
					// �ϥ�GPU�B��
					applyCorrectionMat_gpu(mat_gpu_depth[eye],
						corretion_line[LINE_LEFT].slope, corretion_line[LINE_LEFT].yIntercept, corretion_line[LINE_LEFT].p1.w, corretion_line[LINE_LEFT].p1.h, corretion_line[LINE_LEFT].p2.w, corretion_line[LINE_LEFT].p2.h,
						corretion_line[LINE_RIGHT].slope, corretion_line[LINE_RIGHT].yIntercept, corretion_line[LINE_RIGHT].p1.w, corretion_line[LINE_RIGHT].p1.h, corretion_line[LINE_RIGHT].p2.w, corretion_line[LINE_RIGHT].p2.h,
						corretion_line[LINE_TOP].slope, corretion_line[LINE_TOP].yIntercept, corretion_line[LINE_TOP].p1.w, corretion_line[LINE_TOP].p1.h, corretion_line[LINE_TOP].p2.w, corretion_line[LINE_TOP].p2.h,
						corretion_line[LINE_DOWN].slope, corretion_line[LINE_DOWN].yIntercept, corretion_line[LINE_DOWN].p1.w, corretion_line[LINE_DOWN].p1.h, corretion_line[LINE_DOWN].p2.w, corretion_line[LINE_DOWN].p2.h
						);
					// �M�β`�סA���l�v�����`�פӲ`���a����N��
					applyDepthMat_gpu(mat_gpu_image[eye], mat_gpu_depth[eye]);
				}
				// �v���ƻs
#ifdef D3D_CUDA_INTEROP
				// �b���Ұ�CUDA�PDX11 interop�����p�U�A�i�H�����i��GPU-to-GPU�O����ƻs
				// �ˬd�O�_���i��Ltextrue����l��
				if (textureInit && nativeTexture[eye]){
					// GPU�O����ƻs
					cudaArray_t arrIm;
					cudaGraphicsSubResourceGetMappedArray(&arrIm, cuda_img[eye], 0, 0);
					cudaMemcpy2DToArray(arrIm, 0, 0, mat_gpu_image[eye].data, mat_gpu_image[eye].step, imageWidth * 4, imageHeight, cudaMemcpyDeviceToDevice);
				}
#else
				// �Ϥ��A�p�G�S���ҥΡA����N��GPU��X�Ӫ����G�ƻs�^CPU
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

// ���sZED��tracking�\��A���U����m���s�]�����I
// �D�n�Ω�ե�
void rf_resetTracking(){
	zed->stopTracking(); // ����tracking
	position.setIdentity(4, 4); // ��ثe��m�]�m�����I
	zed->enableTracking(position, true); // �A���Ұ�tracking
}

// ���osocket������ɶ��A�]�t�ǿ�P�����Y
float rf_getSocketDelay(){
	return socket_getDelay();
}
// ��s���ݩж������e(����Box���K��)
int rf_updateRemoteRoom(){
	return remoteRoom_update();
}
// ���o�v���j�p(Bytes)
int rf_getImageSize(){
	return imageSize;
}
// ���o�v���e��(pixel)
int rf_getImageWidth(){
	return imageWidth;
}
// ���o�v������(pixel)
int rf_getImageHeight(){
	return imageHeight;
}
// �פ�e��cleanup�B�z�A���ǭ��n�C�򥻤W�N�Orf_init���A��
void rf_destroy(){
	remoteRoom_destroy();
	socket_destroy();
	texture_destroy();
	zed_destory();
	internal_destroy();
}
// ���o���ݩж��Ϊ��O�������
// �bUnity���|����Ū���o�ӫ��Ъ����e�Ө�s���ݩж������ӭ����K��
// side�G�n���������N��
void* rf_getRemoteRoomTexturePtr(int side){
	return remoteRoomTextureBuffers[remoteRoomTextureBufferIndex][side];
}

// �Ω�S��CUDA-D3D interop�����p
// ��⧹���[��̩ж��v����pointer�e�^��Unity
void* rf_getCulledImagePtr(int eye)
{
	return mat_image[eye].data;
}

// ���oZED��FPS
float rf_getZedFPS(){
	return zed->getCurrentFPS();
}
// �]�w�O�_�n�M�β`��
// result�i�H�O0�P�D0���Ʀr
// �����n�]�w���D0���Ʀr�A�~�|��`�׹ϮM�Ψ��l�v���W
void rf_setApplyDepth(int result){
	apply_depth = result;
}

// �]�w�|��Ϊ��|���I
// position�GRECT_LT�BRECT_LD�BRECT_RT�BRECT_RD�|�Ө���
void rf_setCorrectionPixel(int position, float w, float h){
	correction_point[position].set(w, h);
}

// �p��|��檺�|����A��|���I�]�w�n����A�n�I�s�o�Ө�ƨӺ�X�|����
void rf_computeCorrection(){
	corretion_line[LINE_TOP].setFromPoint(correction_point[RECT_LT], correction_point[RECT_RT]);
	corretion_line[LINE_DOWN].setFromPoint(correction_point[RECT_LD], correction_point[RECT_RD]);
	corretion_line[LINE_LEFT].setFromPoint(correction_point[RECT_LD], correction_point[RECT_LT]);
	corretion_line[LINE_RIGHT].setFromPoint(correction_point[RECT_RD], correction_point[RECT_RT]);
}

// ���o��etracking��m�����СA��Unity�i�H�N���ഫ���@�ɮy�Шt
// �p�G�W��ZED�����\������m(TRACKING_GOOD)�A�h�^�Ǧ�m��ƪ�float����
// �Y�_�A�h�^��NULL
float* rf_getPositionPtr(){
	if (track_state == sl::zed::TRACKING_GOOD){
		positionT = position.transpose(); // �i����m�A�D�n�O�]��Unity�榡�PZED�榡���P
		return positionT.data();
	}
	else{
		return NULL;
	}
}

// ���o�e�����Y�@�I���`��
// �ѼƬO�e���W���G���y��
float rf_getDepth(float w, float h){

	int x = int(w);
	int y = (imageHeight - (int)h - 1); // �ഫY�y�СA�]���}�C�O�W�ӤU�x�s�A�ӧڭ̧Ʊ檺�y�Шt��Y�b�O�U�ӤW
	cout << "Retrieve Depth at X:" << x << ", Y:" << y << endl;
	if (x >= 0 && x < imageWidth && y >= 0 && y < imageHeight){ // �T�w�y�нd�򥿽T�L�~
		sl::zed::Mat mat_depth = zed->retrieveMeasure(sl::zed::DEPTH); // ���o�@�i�`�׹Ϧs��CPU
		return ((float*)mat_depth.data)[y* imageWidth + x]; // �^�Ǹ��I���`�׸�T
	}
	else{
		return 0.0f;
	}
}

// �]�w�n���N���`��threshold�A�^�ǭ쥻��threshold
float rf_setDepthThreshold(float threshold){
	float ret = depthThreshold;
	depthThreshold = threshold;
	return ret;
}
// �˹��U�O�_���ҥ�CUDA-D3D interop�A�D�n��Unity���ӧP�_
int rf_isD3DInterop(){
#ifdef D3D_CUDA_INTEROP
	return TRUE;
#else
	return FALSE;
#endif
}

