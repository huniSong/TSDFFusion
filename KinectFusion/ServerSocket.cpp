#include "ServerSocket.h"
#include <Ws2tcpip.h>
#include <iostream>
#include <vector>
#include <string>
#include<pcl/point_types.h> //PCL��֧�ֵĵ�����ͷ�ļ���

using namespace std;

ServerSocket::ServerSocket() :m_nServerSocket(-1)
{
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
		cout << "Socket�汾����" << endl;
}

ServerSocket::~ServerSocket()
{
	WSACleanup();
	closeMySocket();
}

void ServerSocket::closeMySocket()
{
	if (m_nServerSocket != -1)
		closesocket(m_nServerSocket);	//�ر�socket����

	m_nServerSocket = -1;
	WSACleanup();	//��ֹws2_32.lib��ʹ��
}

bool ServerSocket::createSocket()
{
	if (m_nServerSocket == -1)
	{
		m_nServerSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);	//�趨TCPЭ��ӿ�;ʧ�ܷ���INVALID_SOCKET
		if (m_nServerSocket != INVALID_SOCKET)
		{
			cout << "�����������ɹ�" << endl;
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
		nRet = ::bind(m_nServerSocket, (sockaddr*)&Serveraddr, sizeof(sockaddr_in));	//�󶨷�������ַ�Ͷ˿ں�;�ɹ�������0������ΪSOCKET_ERROR
	}

	if (nRet == 0)
	{
		cout << "��IP�Ͷ˿ڳɹ�" << endl;
		return true;
	}

	cout << "��IP�Ͷ˿�ʧ��,The error code is:" << WSAGetLastError() << endl;
	return false;
}

bool ServerSocket::listenSocket()
{
	int nRet = -1;
	if (m_nServerSocket != -1)
	{
		nRet = listen(m_nServerSocket, 10);//�趨�������ӵ��׽��֣��Լ��趨���Ӷ��г���;�ɹ�����0��ʧ�ܷ���-1
	}
	if (nRet == SOCKET_ERROR)
	{
		cout << "������ʧ��" << endl;
		return false;
	}

	cout << "�����󶨳ɹ�" << endl;
	return true;
}

bool ServerSocket::acceptSocket()
{
	sockaddr_in nClientSocket;
	int nSizeClient = sizeof(nClientSocket);
	while (m_nServerSocket != -1)
	{
		sClient = accept(m_nServerSocket, (sockaddr*)&nClientSocket, &nSizeClient);//���ܿͻ������ӣ�����״̬;ʧ�ܷ���-1
		if (sClient == SOCKET_ERROR)
		{
			cout << "accept����ʧ��" << endl;
			return false;
		}
		else {
			cout << "����һ���ͻ��˳ɹ�" << endl;
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
		std::cout << "��ǰ��ͻ���δ����" << std::endl;
	}
}

void ServerSocket::ReceivePoints(char* buffer, int len, int pSize) {
	int bufferRecv = -1;
	if (sClient != -1) {
		bufferRecv = recv(sClient, buffer, len, 0);
		int nAlreadyRead = bufferRecv;
		while (nAlreadyRead != pSize)
		{
			//�е�ʱ������δ�������ﻺ�������Ѿ���ʼ��ȡ
			int tempSize = pSize - nAlreadyRead;
			char* tempBuffer = new char[tempSize];
			int tempnAlreadyRead = recv(sClient, tempBuffer, tempSize, 0);
			memcpy(buffer + nAlreadyRead, tempBuffer, tempnAlreadyRead);
			nAlreadyRead += tempnAlreadyRead;
			delete[] tempBuffer;
		}
	}
	else {
		std::cout << "��ǰ��ͻ���δ����" << std::endl;
	}
}

void ServerSocket::SendBytes(const char* buf, int len) {
	int resultSend = -1;
	if (sClient != -1) {
		resultSend = send(sClient, buf, len, 0);
		if (resultSend == 0) {
			std::cout << "��ͻ��������жϣ�" << std::endl;
			bConnected = false;
		}
		else if (resultSend < 0)std::cout << "���ʹ���" << std::endl;
	}
	else
	{
		std::cout << "��ǰ��ͻ���δ����" << std::endl;
	}
}