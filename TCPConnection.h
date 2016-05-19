#define WIN32_LEAN_AND_MEAN

#pragma once
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <iostream>

#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")
#pragma once

class CTCPConnection
{
public:
	CTCPConnection();
	~CTCPConnection();

	WSADATA wsaData;
	SOCKET ConnectSocket;
	char *sendbuf;

private:
	int actionResult;
	bool connected;

public:
	inline int isConnected() {return connected;};
	int setupConnection(char* address, char* port);
	int sendData(const char* data);
	int closeConnection();

};

