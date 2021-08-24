// NOTE: This is not part of the library, this file holds examples and tests

#include "../include/NetWrap.h"
#include <iostream>
#include <windows.h>
#pragma comment(lib, "uWS.lib")

int main()
{
	IServer* server = create_server();

	ServerInitParam param;
	param.port = 3000;
	param.onConnected = [](IConnect* c) {
		std::cout << "onConnected!" << std::endl;
	};

	param.onDisconnect = [](IConnect* c) {
		std::cout << "onDisconnect!" << std::endl;
	};

	param.onError = [](int port) {
		std::cout << "onError!" << std::endl;
	};

	param.onRecv = [](IConnect* c,char* data,unsigned int len) {
		std::cout << "onRecv! len:"<< len << std::endl;

		c->Send(data,len);
	};

	server->Init(param);

	std::cout << "listen on 3000!" << std::endl;

	while (true)
	{
		server->Poll();
		::Sleep(33);
	}

	server->Uninit();

	release_server(server);
}