#pragma once

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>

// 傳輸用的預設Buffer大小
#define DEFAULT_BUFLEN 512
// 遠端伺服器資訊
#define DEFAULT_PORT "5566" // Port
#define SERVER_NAME "140.109.23.81" // IP或hostname

// 定義單一筆Header的長度
#define SINGLE_HEADER_SIZE 11


void socket_init();
bool socket_retrieve_image(int buffer_index);
void socket_destroy();
int socket_recv_n(void* buffer, int size);

float socket_getDelay();

std::string GetLastErrorAsString();