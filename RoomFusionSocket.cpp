#include "stdafx.h"
#include "RoomFusionSocket.h"

#include <iostream>
#include <cstdlib>
#include <ctime>



using namespace std;

bool socket_ready = false;
SOCKET ConnectSocket = INVALID_SOCKET;
char header[HEADER_SIZE + 1];
float socketDelay;


extern int remoteBoxDataSize[];
extern unsigned char* remoteRoomTextureBuffers[6];

void socket_init(){
	if (socket_ready){ // do nothing if already ready
		return;
	}
	WSADATA wsaData;
	
	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;
	int iResult;


	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo(SERVER_NAME, DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
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
		printf("Unable to connect to server!\n");
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

float socket_getDelay(){
	return socketDelay;
}

bool socket_retrieve_image(){ // six faces
	int result;
	if (socket_ready){
		
		// send request
		send(ConnectSocket, "1", 1, 0);
		// retrieve header
		result = recv(ConnectSocket, header, HEADER_SIZE, 0);
		if (result > 0){
			int data_size = atoi(header);
			if (data_size == DATA_SIZE){
				clock_t start_time = clock();
				for (int i = 0; i < 6; i++){
					// read 6 face
					int size_remain = remoteBoxDataSize[i];
					int offset = 0;
					while (size_remain > 0){
						int nError = recv(ConnectSocket, (char*)remoteRoomTextureBuffers[i] + offset, size_remain, 0);
						if ((nError == SOCKET_ERROR) || (nError == 0)){
							cout << "Error:" << GetLastErrorAsString() << endl;
							break;
						}
						size_remain -= nError;
						offset += nError;
					}
				}
				socketDelay = ((float)(clock() - start_time)) / CLOCKS_PER_SEC;
				return true;
			}
			else{
				cout << "Incorrect data size: " << data_size << endl;
				return false;
			}
		}
		else{
			// socket error, close socket
			socket_destroy();
			return false;
		}
	}
	else{
		return false;
	}
}

void socket_destroy(){
	if (socket_ready){
		closesocket(ConnectSocket);
		WSACleanup();
		socket_ready = false;
		cout << "Socket Destroyed" << endl;
	}
}