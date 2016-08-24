#include "stdafx.h"
#include "RoomFusionSocket.h"

#include <iostream>
#include <cstdlib>
#include <ctime>

#include <fstream>

#include <lz4.h>
#include <lz4frame.h>


using namespace std;

// Socket相關變數
bool socket_ready = false; // 是否已經初始化
SOCKET ConnectSocket = INVALID_SOCKET;
char headerBuffer[SINGLE_HEADER_SIZE + 1];
float socketDelay; // Socket延遲
int compressedSize[6]; // 壓縮後的大小
unsigned char* compressionBuffer[6]; // 用以儲存壓縮後的資料，size is same as un-compressed data

// 以下是RoomFusionInternal.cpp定義的全域變數，請參閱該份檔案
extern int remoteBoxDataSize[];

extern unsigned char* remoteRoomTextureBuffers[2][6]; 
extern int remoteRoomTextureBufferIndex;
extern bool remoteRoomTextureBufferUpdated;

// 初始化Winsock
void socket_init(){
	if (socket_ready){ // do nothing if already ready
		return;
	}
	// prepare buffer
	for (int i = 0; i < 6; i++){
		compressionBuffer[i] = (unsigned char*)malloc(remoteBoxDataSize[i] * sizeof(unsigned char));
		compressedSize[i] = 0;
	}

	WSADATA wsaData;
	
	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;
	int iResult;


	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		cout << "WSAStartup failed with error : " << iResult << endl;
		return;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(SERVER_NAME, DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		cout << "getaddrinfo failed with error:" << iResult << endl;
		WSACleanup();
		return;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			cout << "socket failed with error:" << GetLastErrorAsString() << endl;
			WSACleanup();
			return;
		}

		// Connect to server.
		iResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET) {
		cout << "Unable to connect to server!" << endl;
		WSACleanup();
		return;
	}
	socket_ready = true;
	socketDelay = 0.0f;

}

//Returns the last Win32 error, in string format. Returns an empty string if there is no error.
std::string GetLastErrorAsString()
{
	//Get the error message, if any.
	DWORD errorMessageID = ::WSAGetLastError();
	if (errorMessageID == 0)
		return std::string(); //No error message has been recorded

	LPSTR messageBuffer = nullptr;
	size_t size = FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL, errorMessageID, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&messageBuffer, 0, NULL);

	std::string message(messageBuffer, size);

	//Free the buffer.
	LocalFree(messageBuffer);

	return message;
}
// 取得socket delat
float socket_getDelay(){
	return socketDelay;
}
// 從socket收滿剛好n byte的資料
int socket_recv_n(void* buffer, int size){
	int offset = 0;
	int remain = size;
	while (remain > 0){
		int nError = recv(ConnectSocket, (char*)buffer + offset, remain, 0);
		if ((nError == SOCKET_ERROR) || (nError == 0)){
			cout << "Receive Error:" << GetLastErrorAsString() << endl;
			return -1;
		}
		remain -= nError;
		offset += nError;
	}
	return size;
}
// 從遠端Server接收並解壓縮六個面的影像，存放到buffer_index所指定的那一個buffer中
bool socket_retrieve_image(int buffer_index){ // six faces
	int result;
	if (socket_ready){
		// 計時
		clock_t start_time = clock();
		// send request，請求取得下一份影像
		send(ConnectSocket, "1", 1, 0);
		// retrieve status
		char statusBuf[3] = {0};
		result = recv(ConnectSocket, statusBuf, sizeof(statusBuf) - 1, 0);
		// 檢查server端是否有影像可用
		if (result <= 0 || statusBuf[0] != '1'){
			// no data
			return false;
		}
		// retrieve header for 6 times
		// 這六個header其實就是六個面壓縮後的大小
		for (int i = 0; i<6; i++){
			result = recv(ConnectSocket, headerBuffer, SINGLE_HEADER_SIZE, 0);
			if (result > 0){
				compressedSize[i] = atoi(headerBuffer);
			}
			else{
				// socket error, close socket
				cout << "Socket Error when receive header" << endl;
				cout << "Error:" << GetLastErrorAsString() << endl;
				socket_destroy();
				return false;
			}
		}
		// receive 6 faces and decompress
		for (int i = 0; i < 6; i++){
			int result = socket_recv_n(compressionBuffer[i] , compressedSize[i]);
			if (result < 0){
				// socket error, close socket
				cout << "Socket Error when receive face:" << i << endl;
				cout << "Error:" << GetLastErrorAsString() << endl;
				socket_destroy();
				return false;
			}
		}
		// decompress 6 faces info corresponding buffer
		for (int i = 0; i < 6 ; i++){
			int result = LZ4_decompress_safe((const char*)compressionBuffer[i], (char*)remoteRoomTextureBuffers[buffer_index][i], compressedSize[i], remoteBoxDataSize[i]);
			if (result < 0){
				cout << "Decompress error for face:" << i << ", error = " << LZ4F_getErrorName(result) << endl;
				return false;
			}

		}
		// 計算socket delay
		socketDelay = ((float)(clock() - start_time)) / CLOCKS_PER_SEC;
		return true;
	}
	else{
		return false;
	}
}
// socket cleanup
void socket_destroy(){
	if (socket_ready){
		closesocket(ConnectSocket);
		WSACleanup();
		// remove buffer
		for (int i = 0; i < 6; i++){
			free(compressionBuffer[i]);
		}
		socket_ready = false;
		cout << "Socket Destroyed" << endl;
	}
}