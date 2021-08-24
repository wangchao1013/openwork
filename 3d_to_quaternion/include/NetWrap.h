

#ifndef NETWARP_H
#define NETWARP_H


#ifdef _WIN32
#define MY_EXPORT __declspec(dllexport)
#else
#define MY_EXPORT
#endif

#include <functional>

//×î¼òµÄnet·â×°

//------------------------------------------------------------------
//server 

class IConnect
{
public:
	virtual void* GetUserData() = 0;
	virtual void  SetUserData(void* data) = 0;
	virtual int   Send(char* data, unsigned int len) = 0;
	virtual int   Close() = 0;
	virtual bool  IsConnected() = 0;
};

struct ServerInitParam
{
	int port;
	std::function<void(IConnect*)> onConnected;
	std::function<void(IConnect*)> onDisconnect;
	std::function<void(IConnect*, char*, unsigned int)> onRecv;
	std::function<void(int)>	   onError;
};

class IServer
{
public:
	virtual int Init(const ServerInitParam& param) = 0;
	virtual void Uninit() = 0;
	virtual void Poll() = 0;
};

#if defined(__cplusplus)
extern "C" {
#endif

MY_EXPORT IServer* create_server();
MY_EXPORT void release_server(IServer* server);

#if defined(__cplusplus)
};
#endif

//------------------------------------------------------------------
//client
class IClient;
struct ClientInitParam
{
	std::function<void(IClient*)> onConnected;
	std::function<void(IClient*)> onDisconnect;
	std::function<void(IClient*, char*, unsigned int)> onRecv;
	std::function<void(void*)>	  onError;
};

class IClient
{
public:
	virtual int  Init(const ClientInitParam& param) = 0;
	virtual int  Connect(const char* addr) = 0;
	virtual void Disconnect() = 0;
	virtual void Poll() = 0;
	virtual int  Send(char* data, unsigned int len) = 0;
	virtual bool IsConnected() = 0;
};

//------------------------------------------------------------------
#if defined(__cplusplus)
extern "C" {
#endif

MY_EXPORT IClient* create_client();
MY_EXPORT void release_client(IClient* client);

#if defined(__cplusplus)
};
#endif



#endif 
