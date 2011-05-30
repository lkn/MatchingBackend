// TODO: this is very specific for use with SURFMatcher right now

#include <fstream>
#include <string>

#include "../support/util.h"
#include "StatsTable.h"

StatsTable::StatsTable() {
	struct tm timeinfo = Util::GetTimeInfo();
	char buffer[80];
	strftime(buffer, 80, "MATCH-%m-%d-%Y.html", &timeinfo);
	html_file_ = new std::ofstream(buffer, std::ios::out);
	(*html_file_) << "<html><head>Match stats</head><body>\n";

	strftime(buffer, 80, "MATCH-%m-%d-%Y.csv", &timeinfo);
	csv_file_ = new std::ofstream(buffer, std::ios::out);

	BeginTable();
}

StatsTable::~StatsTable() {
	EndTable();
	if (html_file_) {
		(*html_file_) << "\n</body></html>\n";
		html_file_->close();
		delete html_file_;
		html_file_ = NULL;
	}

	if (csv_file_) {
		csv_file_->close();
		delete csv_file_;
		csv_file_ = NULL;
	}
}

void StatsTable::BeginTable() {
	(*html_file_) << "<table>\n\t<tr><th>filename</th><th>match %</th><th>match with</th><th>time</th></tr>\n";
}

void StatsTable::WriteRow(string filename, double match_percent, string match_with, double time) {
	(*html_file_) << "\t<tr><td>" << filename << "</td><td>" << match_percent << "</td><td>" << match_with << "</td><td>" << time << "<td></td></tr>\n";
	(*csv_file_) << filename << "," << match_percent << "," << match_with << "," << time << endl;
}

void StatsTable::EndTable() {
	(*html_file_) << "</table>\n";
}