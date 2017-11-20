#pragma once
class UDP_Sock
{
public:
	UDP_Sock();
	~UDP_Sock();

private:
	SOCKET m_socket;
};
