// NOTE: This is not part of the library, this file holds examples and tests

#include "../include/NetWrap.h"
#include <iostream>
#include <fstream>
#include <string>
#include <windows.h>
#include <json.h>

#pragma comment(lib, "uWS.lib")

int main()
{
	IClient* client = create_client();

	ClientInitParam param;
	
	param.onConnected = [](IClient* c) {
		std::cout << "onConnected!" << std::endl;
		// read from json file
		std::string file_name = "./body_quaternion.json";
		std::cout << file_name << std::endl;
		std::ifstream is(file_name, std::ios::binary);
		
		if (!is.is_open()) {
			std::cout << "open json file failed." << std::endl;
		}
		Json::CharReaderBuilder reader;
		Json::Value pose_json;
		JSONCPP_STRING errs;
		Json::parseFromStream(reader, is, &pose_json, &errs);
		
		int json_count = 0;
		// int frame_count = pose_json["frame_count"].asInt();
		while (true){
			// if(frame_count == json_count) json_count = 0;
			std::string file_name = "frame" + std::to_string(json_count);
			Json::StreamWriterBuilder builder;
			// std::string output = Json::writeString(builder, pose_json[file_name]);
			std::string output = Json::writeString(builder, pose_json);
			std::cout << file_name << std::endl;
			char* pose_info = (char*)output.c_str();
			c->Send(pose_info, 8000);
			json_count++;
			::Sleep(33);
		}
	};

	param.onDisconnect = [](IClient* c) {
		std::cout << "onDisconnect!" << std::endl;
	};

	param.onError = [](void* p) {
		std::cout << "onError!" << std::endl;
	};

	param.onRecv = [](IClient* c,char* data,unsigned int len) {
		std::cout << "onRecv! len: "<<len << std::endl;
	};

	client->Init(param);

	std::cout << "connect.... on localhost:3000!" << std::endl;
	client->Connect("ws://127.0.0.1:3000");

	while (true)
	{
		client->Poll();
		::Sleep(33);
	}

	client->Disconnect();
	release_client(client);
}