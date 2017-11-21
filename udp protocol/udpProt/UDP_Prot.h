#ifndef UDP_PROT_HPP
#define UDP_PROT_HPP

#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h> // socket

static const int s_BUF_LEN = 1024;
static const int DEFAULT_PORT = 5432;

class UDP_Server
{
public:
	UDP_Server();
	~UDP_Server();

	bool IsInit() { return m_socket != INVALID_SOCKET; };
	bool Init(int portNum = DEFAULT_PORT);

	int Read(char * buffer);
	bool Write(char * buffer, int size);

private:
	SOCKET m_socket;
	struct sockaddr_in m_siOther;
};


class UDP_Client
{
public:
public:
	UDP_Client();
	~UDP_Client();

	bool IsInit() { return m_socket != INVALID_SOCKET; };
	bool Init(int portNum = DEFAULT_PORT, char *host = "127.0.0.1");

	int Read(char * buffer);
	bool Write(char * buffer, int size);

private:
	SOCKET m_socket;
	struct sockaddr_in m_server;
};

# endif // TCP_PROT_HPP