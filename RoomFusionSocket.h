#pragma once

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "5566"
#define SERVER_NAME "140.109.23.81"
//#define SERVER_NAME "127.0.0.1"


#define SINGLE_HEADER_SIZE 11


void socket_init();
bool socket_retrieve_image(int buffer_index);
void socket_destroy();
int socket_recv_n(void* buffer, int size);

float socket_getDelay();

std::string GetLastErrorAsString();