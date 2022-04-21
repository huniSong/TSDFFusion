#include <WinSock2.h>

#pragma comment(lib, "ws2_32.lib")  //���� ws2_32.dll

class ServerSocket
{
public:
	bool bConnected = false;

	ServerSocket();
	~ServerSocket();

	bool createSocket();//����socket
	bool bindSocket(const char* ip, const unsigned short prot);//�󶨱��ص�ַ�Ͷ˿ں�
	bool listenSocket();//��ʼ������������������ӵȴ�����
	bool acceptSocket();//�����������ӵ����󣬲����ؾ��
	void ReceiveBytes(char* buf, int len);
	void ReceivePoints(char* buf, int len, int pSize);
	void SendBytes(const char* buf, int len);
	void closeMySocket();//�ر�����

private:
	SOCKET m_nServerSocket;//�󶨱��ص�ַ�Ͷ˿ںŵ��׽ӿ�
	SOCKET sClient; //�ͻ��������׽���
	WSADATA wsaData;
};