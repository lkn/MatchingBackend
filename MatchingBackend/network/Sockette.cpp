#include <algorithm>
#include <iostream>
#include <winsock2.h>
#include "Sockette.h"

// Create the socket (ex. server)
Sockette::Sockette(u_short port) : port_(port) {	
	Create();
}

// Wrap a socket (ex. one that was returned from accept())
Sockette::Sockette(SOCKET s) {
	handle_ = s;
	SOCKADDR_IN sadi;
	int size = sizeof(sadi);

	if (getpeername(handle_, (SOCKADDR *) &sadi, &size) == SOCKET_ERROR) {
		std::cerr << "Error constructing Sockette from SOCKET!!\n";	
		switch (WSAGetLastError()) {
			// Something went wrong, and we can't construct 
			case WSANOTINITIALISED:
				std::cerr << "->1\n";
				break;
			case WSAENETDOWN:
				std::cerr << "->2\n";
				break;
			case WSAEFAULT:
				std::cerr << "->3\n";
				break;
			case WSAEINPROGRESS:
				std::cerr << "->4\n";
				break;
			case WSAENOTCONN:
				std::cerr << "->5\n";
				break;
			case WSAENOTSOCK:
				std::cerr << "->6\n";
				break;
		}
		WSACleanup();
		system("PAUSE");
	}

	address_ = ntohl(inet_addr((char *) sadi.sin_addr.S_un.S_addr));
	port_ = sadi.sin_port;
	int tru = 1;  // disable Nagle
	setsockopt(handle_, IPPROTO_TCP, TCP_NODELAY, (char *) &tru, sizeof(tru));
}

// Creates and binds to socket
void Sockette::Create() {
	handle_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (handle_ == INVALID_SOCKET)	{
		std::cout << "Socket creation failed.\n";
		WSACleanup();
		system("PAUSE");
	}
}

void Sockette::StartListening() {
	SOCKADDR_IN serverInf;
	serverInf.sin_family = AF_INET;
	serverInf.sin_addr.s_addr = INADDR_ANY;
	serverInf.sin_port = htons(port_);
	if (bind(handle_, (SOCKADDR *) (&serverInf), sizeof(serverInf)) == SOCKET_ERROR) {
		std::cout << "Unable to bind socket!\n";
		WSACleanup();
		system("PAUSE");
	}
	listen(handle_, 5);  // TODO: make configurable
}

// first 3 bytes is header.
// Expects that whoever is sending data is using the first 4 bytes
// to encode how many bytes we should expect to receive.
// next byte is the type of request
// Whoever sent in 'data' must deallocate it (this function allocates it)
bool Sockette::Listen(unsigned char *cmd, char **data) {
	std::cout << "Listening...." << std::endl;
	unsigned char * tmp = (unsigned char *) calloc(5, sizeof(unsigned char));
	if (tmp == NULL) std::cerr << "calloc returned NULL";

	// get expected size of data
	int expectedSize = 0;
	while (true) {
		int n = recv(handle_, (char *) tmp, 5, 0);
		std::cout << "Received data from client!" << std::endl;
		std::cout << "error: " << WSAGetLastError() << std::endl;
		if (n == SOCKET_ERROR) {
			if (WSAGetLastError() == WSAEWOULDBLOCK) continue;
			std::cerr << "Error getting packet size: " << WSAGetLastError() << std::endl;
			return false;
		}

		std::cout << "header size: " << n << std::endl;
		
		expectedSize = tmp[0] << 24 | tmp[1] << 16 | tmp[2] << 8 | tmp[3];  // assuming big endian
		std::cout << "Expected size of data: " << expectedSize << std::endl;
		if (expectedSize <= 0) {
			std::cerr << "Something funny with client... we expect 0 bytes. Ignoring and quitting." << std::endl;
			return false;
		}

		*cmd = tmp[4];
		std::cout << "cmd = " << *cmd << std::endl;
		break;
	}

	*data = new char[expectedSize];  // specifically this to keep OpenCV happy
	int numBytesRead = 0;
	while (numBytesRead < expectedSize) {
		int r = recv(handle_, (char *) (*data + numBytesRead), expectedSize - numBytesRead, 0);

		if (r == SOCKET_ERROR) {
			if (WSAGetLastError() == WSAEWOULDBLOCK) continue;
			std::cerr << "Couldn't receive, error: " << WSAGetLastError() << std::endl;
			return false;
		}
		numBytesRead += r;
	}
	std::cout << "Actually read " << numBytesRead << std::endl;
	delete tmp;

	// something funky happened, we'll just assume client is in some weird state
	if (numBytesRead == 0) {
		std::cerr << "Something funny with client... we read 0 bytes. Ignoring and quitting." << std::endl;
		return false;
	}

	return true;
}

// TODO: make it work for large amounts of data
bool Sockette::Send(std::string data) {
	std::cout << "Sending " << data << std::endl;
	if (data.empty()) {
		std::cerr << "why you tryna send an empty string?\n";
		return false;
	}
	// TODO: hack... remove newlines in data sent in
	// also.. we just add a new line since its easy for java client since it uses in.readLine()
	
	data.erase(std::remove(data.begin(), data.end(), '\n'), data.end());
	data.append("\n");
	int sent = send(handle_, data.c_str(), data.length(), 0);
	if (sent == SOCKET_ERROR) {
		std::cerr << "Error sending packet! " << WSAGetLastError() << std::endl;
		return false;
	}
	std::cout << "Number of bytes sent to client: " << sent << std::endl;
	return true;
}

Sockette::~Sockette() {
	// TODO shutdown(handle_);
	closesocket(handle_);
}
