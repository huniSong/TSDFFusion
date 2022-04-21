#include "ServerSocket.h"
#include <Ws2tcpip.h>
#include <iostream>
#include <vector>
#include <string>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。

using namespace std;

ServerSocket::ServerSocket() :m_nServerSocket(-1)
{
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
		cout << "Socket版本错误" << endl;
}

ServerSocket::~ServerSocket()
{
	WSACleanup();
	closeMySocket();
}

void ServerSocket::closeMySocket()
{
	if (m_nServerSocket != -1)
		closesocket(m_nServerSocket);	//关闭socket连接

	m_nServerSocket = -1;
	WSACleanup();	//终止ws2_32.lib的使用
}

bool ServerSocket::createSocket()
{
	if (m_nServerSocket == -1)
	{
		m_nServerSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);	//设定TCP协议接口;失败返回INVALID_SOCKET
		if (m_nServerSocket != INVALID_SOCKET)
		{
			cout << "服务器启动成功" << endl;
			return true;
		}
	}
	return false;
}

bool ServerSocket::bindSocket(const char* ip, const unsigned short prot)
{
	int nRet = -1;
	if (m_nServerSocket != -1)
	{
		sockaddr_in Serveraddr;
		memset(&Serveraddr, 0, sizeof(sockaddr_in));
		Serveraddr.sin_family = AF_INET;
		Serveraddr.sin_addr.S_un.S_addr = htonl(INADDR_ANY);
		//inet_pton(AF_INET, ip, &Serveraddr.sin_addr.s_addr);
		Serveraddr.sin_port = htons(prot);
		nRet = ::bind(m_nServerSocket, (sockaddr*)&Serveraddr, sizeof(sockaddr_in));	//绑定服务器地址和端口号;成功，返回0，否则为SOCKET_ERROR
	}

	if (nRet == 0)
	{
		cout << "绑定IP和端口成功" << endl;
		return true;
	}

	cout << "绑定IP和端口失败,The error code is:" << WSAGetLastError() << endl;
	return false;
}

bool ServerSocket::listenSocket()
{
	int nRet = -1;
	if (m_nServerSocket != -1)
	{
		nRet = listen(m_nServerSocket, 10);//设定接受连接的套接字，以及设定连接队列长度;成功返回0，失败返回-1
	}
	if (nRet == SOCKET_ERROR)
	{
		cout << "监听绑定失败" << endl;
		return false;
	}

	cout << "监听绑定成功" << endl;
	return true;
}

bool ServerSocket::acceptSocket()
{
	sockaddr_in nClientSocket;
	int nSizeClient = sizeof(nClientSocket);
	while (m_nServerSocket != -1)
	{
		sClient = accept(m_nServerSocket, (sockaddr*)&nClientSocket, &nSizeClient);//接受客户端连接，阻塞状态;失败返回-1
		if (sClient == SOCKET_ERROR)
		{
			cout << "accept连接失败" << endl;
			return false;
		}
		else {
			cout << "连接一个客户端成功" << endl;
			bConnected = true;
			return true;
		}
		
	}
	return false;
}

void ServerSocket::ReceiveBytes(char* buffer, int len) {
	int bufferRecv = -1;
	if (sClient != -1) {
		bufferRecv = recv(sClient, buffer, len, 0);
	}
	else {
		std::cout << "当前与客户端未连接" << std::endl;
	}
}

void ServerSocket::ReceivePoints(char* buffer, int len, int pSize) {
	int bufferRecv = -1;
	if (sClient != -1) {
		bufferRecv = recv(sClient, buffer, len, 0);
		int nAlreadyRead = bufferRecv;
		while (nAlreadyRead != pSize)
		{
			//有的时候数据未完整到达缓冲区就已经开始读取
			int tempSize = pSize - nAlreadyRead;
			char* tempBuffer = new char[tempSize];
			int tempnAlreadyRead = recv(sClient, tempBuffer, tempSize, 0);
			memcpy(buffer + nAlreadyRead, tempBuffer, tempnAlreadyRead);
			nAlreadyRead += tempnAlreadyRead;
			delete[] tempBuffer;
		}
	}
	else {
		std::cout << "当前与客户端未连接" << std::endl;
	}
}

void ServerSocket::SendBytes(const char* buf, int len) {
	int resultSend = -1;
	if (sClient != -1) {
		resultSend = send(sClient, buf, len, 0);
		if (resultSend == 0) {
			std::cout << "与客户端连接中断！" << std::endl;
			bConnected = false;
		}
		else if (resultSend < 0)std::cout << "发送错误！" << std::endl;
	}
	else
	{
		std::cout << "当前与客户端未连接" << std::endl;
	}
}