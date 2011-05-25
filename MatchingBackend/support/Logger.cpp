#include <iostream>
#include <time.h>
#include <stdarg.h>
#include <time.h>

#include "../support/util.h"
#include "Logger.h"

Logger::Logger() {
	Init("");
}

Logger::Logger(std::string fileName) {
	Init(fileName);
}

Logger::~Logger() {
	logFile_->close();
	delete logFile_;
	logFile_ = NULL;
}

void Logger::Init(std::string fileName) {
	if (fileName.empty()) {
		std::cerr << "Empty file name to logger" << std::endl;
		struct tm timeinfo = Util::GetTimeInfo();
		char buffer[80];
		strftime(buffer, 80, "LOG-%m-%d-%Y.txt", &timeinfo);
		fileName = buffer;
	}

	logFile_ = new std::ofstream(fileName, std::ios::out | std::ios::app);
	(*logFile_) << "\n\n-------------------------------------------\n";
}

// Will also write to file
void Logger::Log(LogLevel level, const char *format, ...) {
	char buffer[1024];
	va_list args;
	va_start(args, format);
	vsnprintf(buffer, 1024, format, args);
	va_end(args);
	
	char timeBuffer[10];
	struct tm timeinfo = Util::GetTimeInfo();
	strftime(timeBuffer, 10, "%I:%M:%S", &timeinfo);
	
	char logString[1024];
	sprintf(logString, "[%s] %s\n", timeBuffer, buffer);

	if (logFile_ && logFile_->is_open()) {
		(*logFile_) << logString;
		logFile_->flush();
	}

	std::cout << logString;
}


