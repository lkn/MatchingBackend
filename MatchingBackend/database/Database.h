#ifndef DATABASE_H__
#define DATABASE_H__

#include <string>
#include <vector>

#include "../thirdparty/sqlite3/sqlite3.h"

using namespace std;

typedef vector<string> DbRow;

class Database {
public:
	Database(const char* filename);
	~Database();

	vector<DbRow> query(const char* query);

protected:
	bool open(const char* filename);
	void close();
	
private:
	sqlite3 *database;
};

#endif

