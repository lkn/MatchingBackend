#include <iostream>

#include "Database.h"

Database::Database(const char* filename) {
	database = NULL;
	open(filename);
}

Database::~Database() {
	if (database != NULL) {
		close();
	}
}

bool Database::open(const char* filename) {
	if (sqlite3_open(filename, &database) == SQLITE_OK) {
		return true;
	}
		
	return false;   
}

vector<vector<string> > Database::query(const char* query) {
	sqlite3_stmt *statement;
	vector<DbRow> results;

	if (sqlite3_prepare_v2(database, query, -1, &statement, 0) == SQLITE_OK) {
		int cols = sqlite3_column_count(statement);
		int result = 0;
		while (true) {
			result = sqlite3_step(statement);
			
			if (result == SQLITE_ROW) {
				vector<string> values;
				for (int col = 0; col < cols; col++)	{
					const unsigned char *text = sqlite3_column_text(statement, col);
					if (text != NULL) {
						values.push_back((char *) text);
					} else {
						values.push_back("");
					}
				}
				results.push_back(values);
			} else {
				break;   
			}
		}
	   
		sqlite3_finalize(statement);
	}
	
	string error = sqlite3_errmsg(database);
	if (error != "not an error") cout << query << " " << error << endl;
	
	return results;  
}

void Database::execute(const char *stmt) {
		std::vector<std::string> vcol_head;
		std::vector<std::string> vdata;
		char *zErrMsg;
	    char **result;
	    int nrow,ncol;
	    
		int rc = sqlite3_get_table(
  			database,  //An open database
  			stmt,      // SQL to be executed
  			&result,   // Result written to a char *[]  that this points to
  			&nrow,     // Number of result rows written here
  			&ncol,     // Number of result columns written here
  			&zErrMsg   // Error msg written here
  			);

        if (vcol_head.size() < 0) { vcol_head.clear(); }
        if (vdata.size() < 0)     { vdata.clear(); }


       if (rc == SQLITE_OK) {
       	  for (int i = 0; i < ncol; ++i)
  			vcol_head.push_back(result[i]);   /* First row heading */
        	for(int i=0; i < ncol*nrow; ++i)
  				vdata.push_back(result[ncol+i]);
       	}

	   sqlite3_free_table(result);
}

void Database::insertComment(const string& user, const string& blurb, const string& imageId) {
	cout << "db insert comment: " << user << " " << blurb << " " << imageId << endl;
	execute(("INSERT into COMMENT (commenter, blurb, image_id) values (\"" + user + "\", \"" + blurb + "\", " + imageId + ")").c_str());
}

void Database::close() {
	sqlite3_close(database);   
}
