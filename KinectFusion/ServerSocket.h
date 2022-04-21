#include <WinSock2.h>

#pragma comment(lib, "ws2_32.lib")  //加载 ws2_32.dll

class ServerSocket
{
public:
	bool bConnected = false;

	ServerSocket();
	~ServerSocket();

	bool createSocket();//创建socket
	bool bindSocket(const char* ip, const unsigned short prot);//绑定本地地址和端口号
	bool listenSocket();//初始化监听并设置最大连接等待数量
	bool acceptSocket();//接受请求连接的请求，并返回句柄
	void ReceiveBytes(char* buf, int len);
	void ReceivePoints(char* buf, int len, int pSize);
	void SendBytes(const char* buf, int len);
	void closeMySocket();//关闭连接

private:
	SOCKET m_nServerSocket;//绑定本地地址和端口号的套接口
	SOCKET sClient; //客户端连接套接字
	WSADATA wsaData;
};