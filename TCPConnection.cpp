#include "TCPConnection.h"



CTCPConnection::CTCPConnection()
{
	connected = false;
	actionResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	/*
	if (state != 0)
	{
		std::cout << "Wystpail blad inicjalizacji WSA: " << actionResult << std::endl;
	}
	*/
}

CTCPConnection::~CTCPConnection()
{
	if (connected)
		closeConnection();
}

int CTCPConnection::setupConnection(char* address, char* port)
{
	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	actionResult = getaddrinfo(address, port, &hints, &result);
	if (actionResult != 0)
	{
		std::cout << "Wystapil blad podczas uzyskiwania parametrow adresu: " << actionResult << std::endl;
		WSACleanup();
		return 0;
	}

	for (ptr = result; ptr != NULL; ptr = ptr->ai_next)
	{
		ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocket == INVALID_SOCKET)
		{
			std::cout << "Wystapil blad socket'a: " << WSAGetLastError() << std::endl;
			WSACleanup();
			return 0;
		}

		actionResult = connect(ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (actionResult == SOCKET_ERROR)
		{
			closesocket(ConnectSocket);
			ConnectSocket = INVALID_SOCKET;
			continue;
		}
		break;
	}

	freeaddrinfo(result);

	if (ConnectSocket == INVALID_SOCKET)
	{
		std::cout << "Nie udalo sie polaczyc - zly socket";
		WSACleanup();
		return 0;
	}

	connected = true;
	return 1;
}

int CTCPConnection::sendData(const char* data)
{
	if (!connected)
	{
		std::cout << "Najpierw nalezy ustanowic polaczenie!" << std::endl;
		return 0;
	}
	actionResult = send(ConnectSocket, data, (int)strlen(data), 0);
	if (actionResult == SOCKET_ERROR)
	{
		std::cout << "Wystapil blad przesylania: " << WSAGetLastError() << std::endl;
		closesocket(ConnectSocket);
		WSACleanup();
		return 0;
	}

	return 1;
}

int CTCPConnection::closeConnection()
{
	if (!connected)
	{
		std::cout << "Najpierw nalezy ustanowic polaczenie!" << std::endl;
		return 0;
	}
	actionResult = shutdown(ConnectSocket, SD_SEND);
	if (actionResult == SOCKET_ERROR)
	{
		std::cout << "Wystapil blad zamykania polaczenia: " << WSAGetLastError() << std::endl;
		closesocket(ConnectSocket);
		WSACleanup();
		return 0;
	}

	connected = false;
	return 1;
}
