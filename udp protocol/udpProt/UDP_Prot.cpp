#include "UDP_Prot.h"

#include <Ws2tcpip.h>
#include <iostream>


static bool InitWinSock()
{
	WSADATA wsaData;

	memset(&wsaData, 0, sizeof(wsaData));

	// init winsock
	int ret = WSAStartup(MAKEWORD(2, 2), &wsaData);

	return 0 == ret;
}

UDP_Server::UDP_Server()
: m_socket(INVALID_SOCKET)
, m_siOther()
{
}

UDP_Server::~UDP_Server()
{
	closesocket(m_socket);
}

bool UDP_Server::Init(int portNum)
{
	
	memset(&m_siOther, 0, sizeof(m_siOther));

	if (!InitWinSock())
	{
		std::cout << "error in init winsock\n";
		return false;
	}

	m_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if (m_socket == INVALID_SOCKET)
	{
		std::cout << "error in socket\n";
		return false;
	}

	struct sockaddr_in server;
	memset(&server, 0, sizeof(server));
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons(portNum);

	if (bind(m_socket, (struct sockaddr *)&server, sizeof(server)) == SOCKET_ERROR)
	{
		std::cout << "error in bind\n";
		return false;
	}

	std::cout << "bind done\n";
	return true;
}

int UDP_Server::Read(char * buffer)
{
	static int slen = sizeof(m_siOther);

	return recvfrom(m_socket, buffer, s_BUF_LEN, 0, (struct sockaddr *) &m_siOther, &slen);
}

bool UDP_Server::Write(char * buffer, int size)
{
	int stat = sendto(m_socket, buffer, size, 0, (struct sockaddr *) &m_siOther, sizeof(m_siOther));

	return stat != SOCKET_ERROR;
}

UDP_Client::UDP_Client()
: m_socket(INVALID_SOCKET)
, m_server()
{
}

UDP_Client::~UDP_Client()
{
	closesocket(m_socket);
}
bool UDP_Client::Init(int portNum, char *host)
{
	if (!InitWinSock())
		return false;

	m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (m_socket == SOCKET_ERROR)
	{
		std::cout << "error in socket\n";
		return false;
	}

	memset(&m_server, 0, sizeof(m_server));
	m_server.sin_family = AF_INET;
	m_server.sin_port = htons(portNum);
	ULONG *srcAddr = new ULONG;
	InetPton(AF_INET, host, srcAddr);
	m_server.sin_addr.S_un.S_addr = *srcAddr;

	std::cout << "client init done\n";
	delete srcAddr;
	return true;
}

int UDP_Client::Read(char * buffer)
{
	static int slen = sizeof(struct sockaddr_in);
	return recvfrom(m_socket, buffer, s_BUF_LEN, 0, (struct sockaddr *) &m_server, &slen);
}

bool UDP_Client::Write(char * buffer, int size)
{
	int stat = sendto(m_socket, buffer, size, 0, (struct sockaddr *) &m_server, sizeof(m_server));
	if (stat <= 0)
		return false;

	return true;
}

