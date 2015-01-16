/*
 * Dao.h
 *
 *  Created on: 2012-09-12
 *      Author: Frank
 */

#ifndef DAO_H_
#define DAO_H_

#include <boost/thread.hpp>
#include <sqlite3.h>
#include <utilite/UtiLite.h>

/**
 * This generic class is use to interface with a database and a mapped object
 */
template <class T>
class Dao {
public:
	/**
	 * Constructor
	 * initialize with synchronous pragma off for faster writing
	 * @param db: sqlite connection object
	 */
	Dao(sqlite3* db, std::string tableName):tableName(tableName)
	{
		dbPtr = db;

		int rc;
		std::string request = "PRAGMA synchronous = OFF";
		rc = sqlite3_exec(db, request.c_str(), 0, 0, 0);
		if(rc != SQLITE_OK)
		{
			UERROR("DB error (rc=%d): %s", rc, sqlite3_errmsg(db));
			UERROR("The query is: %s",request.c_str());
		}
	}
	/**
	 * Destructor
	 */
	virtual ~Dao(){;}
	/**
	 * This function is used to insert a new objet in the database
	 * @param obj: the shared ptr to insert
	 * @return true if operation succeed, false otherwise
	 */
	virtual bool insertObj(boost::shared_ptr<T> obj) = 0;
	/**
	 * This function deletes an object in the database
	 * @param obj: the object to delete
	 * @return true if operation succeed, false otherwise
	 */
	virtual bool deleteObj(boost::shared_ptr<T> obj) = 0;
	/**
	 * This function update an existing object. First it search if the object exist, then it updates
	 * @param obj: the object to update
	 * @return true if operation succeed, false otherwise
	 */
	virtual bool updateObj(boost::shared_ptr<T> obj) = 0;
	/**
	 * This function search a Input object specified by an id
	 * @param id: the identifiant of the Input object
	 * @return a shared pointer to the retrieved Input object
	 */
	virtual boost::shared_ptr<T> find(int uid) = 0;


	/**
	 * Getter function , pointer to sqlite connection object
	 */
	sqlite3* getDbPtr(){return dbPtr;}
	/**
	 * This function clears the data in the selected table, leaving the column definition
	 * @return true if operation succeed, false otherwise
	 */
	virtual bool clearTable() = 0;

protected:
	/**
	 * This function clears the data in the table by specifying a table name, leaving the column definition
	 * @param table: table name
	 * @return true if operation succeed, false otherwise
	 */
	bool truncate()
	{
		bool success = true;
		std::string request;
		sqlite3_stmt* prepared_statement;
		int errorCode = 0;

		request = "DELETE FROM ";
		request = request.append(tableName);

		const char* unusedStatement;
		errorCode = sqlite3_prepare_v2(getDbPtr(), request.c_str(), -1, &prepared_statement, &unusedStatement);

		success &= (errorCode == SQLITE_OK);
		if(success)
		{
			errorCode = sqlite3_step(prepared_statement);
		}
		success &= (errorCode == SQLITE_DONE);

		errorCode = sqlite3_finalize(prepared_statement);
		success &= (errorCode == SQLITE_OK);

		if(!success)
		{
			UERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
			UERROR("The query is: %s",request.c_str());
		}

		return success;
	}

private :
	sqlite3* dbPtr;
	std::string tableName;

};

#endif /* DAO_H_ */

