#include <vector>
#include <iostream>
#include <winsock2.h>
#include "stdafx.h"

#include <opencv2/highgui/highgui.hpp>

#include "thirdparty/tinyxml/tinyxml.h"

#include "database/Database.h"
#include "matching/SURFMatcher.h"
#include "network/Sockette.h"
#include "support/Logger.h"
#include "support/util.h"

#pragma comment(lib, "ws2_32.lib")

using namespace std;

#define PORT 1111
#define DEFAULT_BUFLEN 65536

Database *database = NULL;
Logger *g_logger = NULL;
SURFMatcher *g_matcher = NULL;

QueryParams g_queryParams;

// Deal with client
DWORD WINAPI ClientLoop(LPVOID sockette) {
	Sockette * clientSocket = (Sockette *) sockette;
	g_logger->Log(INFO, "New client (%lu) using port %u doing work!\n",
					    clientSocket->address(), clientSocket->port());

	CvMemStorage *storage = cvCreateMemStorage(0);
	CvSURFParams params = cvSURFParams(g_queryParams.hessian_threshold, g_queryParams.extended_parameter);

	int c = 0;
	while (true) {
		char *dataReceived = NULL;
		string dataToSend;
		// TODO: get rid of memory leaks
		if (clientSocket->Listen(&dataReceived)) {
			// Converting jpg to iplimage... 
			CvMat cvmat = cvMat(g_queryParams.image_width, g_queryParams.image_height, CV_8UC3, (void *) dataReceived);
			IplImage *frame = cvDecodeImage(&cvmat, 1);  // TODO: need to release?
			
			// actual image is in probably color (3 channels)
			// but we need to use 1 channel to do surfmatching which uses grayscale
			// images (1 channel)
			IplImage *queryImage = cvCreateImage(cvSize(g_queryParams.image_width, g_queryParams.image_height), IPL_DEPTH_8U, 1);
			cvCvtColor(frame, queryImage, CV_BGR2GRAY);

			struct tm timeinfo = Util::GetTimeInfo();
			char queryName[128];
			strftime(queryName, 128, "%m-%d-%y-%H-%M-%S.jpg", &timeinfo);
			std::cout << "saving to file " << queryName << endl;
			cvSaveImage(queryName, queryImage);
			
			//clientSocket->Send(queryName);
			CvSeq *queryKeyPoints = 0, *queryDescriptors = 0;
			double tt = (double) -cvGetTickCount();
			cvExtractSURF(queryImage, 0, &queryKeyPoints, &queryDescriptors, storage, params);
			printf("Query Descriptors %d\nQuery Extraction Time = %gm\n",
				queryDescriptors->total, ((tt + cvGetTickCount()) / cvGetTickFrequency()*1000.));
			
			dataToSend = g_matcher->MatchAgainstLibrary(queryName, queryImage, queryKeyPoints, queryDescriptors);
			if (dataToSend.empty()) {
				std::cout << "NO MATCH!\n";
			} else {
				clientSocket->Send(dataToSend);
			}
		} else {
			break;
		}
	}
	std::cout << "Client peaceing out\n";
	delete clientSocket;
	cvReleaseMemStorage(&storage);
	return 0;
}

bool parseSettingsXml(const string& settingsXmlPath, string& logName, QueryParams& queryParams, SURFMatcherParams& libraryParams) {
	
	if (settingsXmlPath.empty()) {
		cerr << "No settings xml path!" << endl;
		return false;
	}

	const char *tmp = NULL;
	TiXmlDocument *doc = new TiXmlDocument(settingsXmlPath.c_str());
	doc->LoadFile();
	TiXmlElement *settingsElement = doc->FirstChildElement("settings");
	TiXmlElement *queryElement = settingsElement->FirstChildElement("query");

	TiXmlElement *surfElement = queryElement->FirstChildElement("surf");
	queryParams.hessian_threshold = atof(surfElement->FirstChildElement("hessian_threshold")->FirstChild()->ToText()->Value());
	queryParams.extended_parameter = atoi(surfElement->FirstChildElement("extended_parameter")->FirstChild()->ToText()->Value());

	TiXmlElement *imageElement = queryElement->FirstChildElement("image");
	queryParams.image_width = atoi(imageElement->FirstChildElement("width")->FirstChild()->ToText()->Value());
	queryParams.image_height = atoi(imageElement->FirstChildElement("height")->FirstChild()->ToText()->Value());

	TiXmlElement *libraryElement = settingsElement->FirstChildElement("library");
	if ((tmp = libraryElement->Attribute("path")) == NULL) {
		cerr << "No library file!" << endl;
		return false;
	} 

	libraryParams.db_path = string(tmp);

	surfElement = libraryElement->FirstChildElement("surf");
	libraryParams.hessian_threshold = atof(surfElement->FirstChildElement("hessian_threshold")->FirstChild()->ToText()->Value());
	libraryParams.extended_parameter = atoi(surfElement->FirstChildElement("extended_parameter")->FirstChild()->ToText()->Value());

	imageElement = libraryElement->FirstChildElement("image");
	libraryParams.image_width = atoi(imageElement->FirstChildElement("width")->FirstChild()->ToText()->Value());
	libraryParams.image_height = atoi(imageElement->FirstChildElement("height")->FirstChild()->ToText()->Value());

	libraryParams.match_threshold = atof(libraryElement->FirstChildElement("match_threshold")->FirstChild()->ToText()->Value());
	
	const char *name = settingsElement->FirstChildElement("log")->Attribute("name");
	if (name != NULL) {
		logName = string(name);
	}
	return true;
}

void readPathTuplesFromDb(vector<pair<string, string> >& pathTuples) {
	vector<DbRow> rows = database->query("SELECT name, path FROM imagedata");
	for (vector<DbRow>::iterator it = rows.begin(); it != rows.end(); ++it) {
		DbRow row = *it;
		pathTuples.push_back(make_pair(row.at(0), row.at(1)));
	}
}

int _tmain(int argc, char *argv[]) {
/*
	if (argc < 2) {
		cerr << "Need to specify settings xml path!" << endl;
		system("PAUSE");
		return 0;
	}
	
	cout << "Settings path: " << argv[1] << endl;
*/
	string logName;
	SURFMatcherParams libraryParams;

	string settingsXmlPath = "settings_server.xml";
	parseSettingsXml(settingsXmlPath, logName, g_queryParams, libraryParams);
	g_logger = new Logger(logName);

	// build library
	g_matcher = new SURFMatcher(g_logger, libraryParams);
	vector<pair<string, string> > pathTuples;
	database = new Database(libraryParams.db_path.c_str());
	readPathTuplesFromDb(pathTuples);
	g_matcher->BuildFromList(pathTuples);

	WSADATA WsaDat;
	if (WSAStartup(MAKEWORD(2, 2), &WsaDat) != 0) {
		std::cout << "WSA Initialization failed!\r\n";
		WSACleanup();
		system("PAUSE");
		return 0;
	}
	
	Sockette *serverSock = new Sockette((u_short) PORT);
	serverSock->StartListening();

	vector<HANDLE> threads = vector<HANDLE>();
	while (true) {
		SOCKET clientSock = SOCKET_ERROR;
		std::cout << "Waiting for incoming connections...\n";
		clientSock = accept(serverSock->handle(), NULL, NULL);
		if (clientSock == INVALID_SOCKET || clientSock == SOCKET_ERROR) {
			cerr << "Error: Accepted bad socket!\n" << endl;
			continue;
		}

		cout << "We have a request!\r\n\r\n" << endl;

		//u_long iMode = 1;  // set non-blocking mode
		//if (ioctlsocket(clientSock, FIONBIO, &iMode)) {
		//	std::cerr << "Error: " << WSAGetLastError();
		//}

		// branch off a thread to deal with the new connection
		Sockette * newSockette = new Sockette(clientSock);  // TODO: delete these

		HANDLE clientThread = CreateThread(0, 0, ClientLoop, (LPVOID) newSockette, 0, 0);
		if (clientThread == NULL) {
			cerr << "Not accepting incoming connections...\n";
			// We couldn't spin off a thread to deal with this connection
			WSACleanup();
			system("PAUSE");
		} else {
			threads.push_back(clientThread);  // TODO: clean these up
		}
	}

	delete database;
	WSACleanup();
	system("PAUSE");
	return 0;
}
