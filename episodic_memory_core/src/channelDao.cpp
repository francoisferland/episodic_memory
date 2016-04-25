#include "channelDao.h"


ChannelDao::ChannelDao(sqlite3* db) : Dao<ChannelObj>(db,"channel")
{
	bool success = true;
	sqlite3_stmt* prepared_statement;
    const char* unusedStatement;
	int errorCode = 0;

    std::string request =
        "create table if not exists channel "
        "(uid INTEGER PRIMARY KEY,"
        " layerId smallint,"
        " relevance double,"
        " indexChannel smallint,"
        " description varchar(100))";

	errorCode = sqlite3_prepare_v2(getDbPtr(), request.c_str(), -1, &prepared_statement, &unusedStatement);

	success &= (errorCode == SQLITE_OK);
	if(success)
	{
		errorCode = sqlite3_step(prepared_statement);
	}
	success &= (errorCode == SQLITE_DONE);

	sqlite3_finalize(prepared_statement);

	if(!success)
	{
		ROS_ERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}
}

ChannelDao::~ChannelDao()
{
}

bool ChannelDao::insertObj(ChannelObjPtr obj)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;


	buf << "INSERT INTO channel ( indexChannel, description, relevance, layerId) "
			<<" VALUES (" << obj->getIndexChannel() << ", '" << obj->getDescription() << "', " <<
			obj->getRelevance() << ", " << obj->getLayerId() << ")";

	request = buf.str();
	const char* unusedStatement;
	errorCode = sqlite3_prepare_v2(getDbPtr(), request.c_str(), -1, &prepared_statement, &unusedStatement);

	success &= (errorCode == SQLITE_OK);
	if(success)
	{
		errorCode = sqlite3_step(prepared_statement);
	}
	success &= (errorCode == SQLITE_DONE);

	sqlite3_finalize(prepared_statement);

	if(success)
	{
		long lastId = sqlite3_last_insert_rowid(getDbPtr());
		obj->setUid(lastId);
	}
	if(!success)
	{
		ROS_ERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}


	return success;
}

bool ChannelDao::deleteObj(ChannelObjPtr obj)
{
	return false;
}

bool ChannelDao::updateObj(ChannelObjPtr obj)
{
	bool success = true;
	bool found = false;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = -1;
	//try to find the object in the table before updating
	if(this->find(obj->getUid()))
		found = true;
	else
		found = false;

	if(found)
	{
		buf << "UPDATE channel SET description='" << obj->getDescription() << "', relevance="<<obj->getRelevance() <<
				", layerId=" << obj->getLayerId() << ", indexChannel=" << obj->getIndexChannel() << " WHERE uid=" << obj->getUid();
		request = buf.str();
		const char* unusedStatement;
		errorCode = sqlite3_prepare_v2(getDbPtr(), request.c_str(), -1, &prepared_statement, &unusedStatement);
	}
	success &= (errorCode == SQLITE_OK);
	if(success)
	{
		errorCode = sqlite3_step(prepared_statement);
	}
	success &= (errorCode == SQLITE_DONE);

	if(found)
	{
		errorCode = sqlite3_finalize(prepared_statement);
		success &= (errorCode == SQLITE_OK);
		if(!success)
		{
			ROS_ERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
			ROS_ERROR("The query is: %s",request.c_str());
		}
	}

	return success;
}

ChannelObjPtr loadChannel(int cols , sqlite3_stmt* prepared_statement)
{
	ChannelObjPtr channel = ChannelObjPtr(new ChannelObj());
	for( int col = 0; col < cols; col++)
	{
		std::string columnName = sqlite3_column_name(prepared_statement, col);
		if(columnName.compare("indexChannel") == 0)
		{
			channel->setIndexChannel((int)sqlite3_column_int(prepared_statement, col));
		}
		else if(columnName.compare("description") == 0)
		{
			channel->setDescription((const char*)sqlite3_column_text(prepared_statement, col));
		}
		else if(columnName.compare("relevance") == 0)
		{
			channel->setRelevance((float)sqlite3_column_double(prepared_statement, col));
		}
		else if(columnName.compare("layerId") == 0)
		{
			channel->setLayerId((int)sqlite3_column_int(prepared_statement, col));
		}
		else if(columnName.compare("uid") == 0)
		{
			channel->setUid((int)sqlite3_column_int(prepared_statement, col));
		}
	}

	return channel;
}

ChannelObjPtr ChannelDao::find(int uid)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	ChannelObjPtr channel;

	buf << "SELECT * FROM channel WHERE uid=" << uid  ;
	request = buf.str();
	const char* unusedStatement;
	errorCode = sqlite3_prepare_v2(getDbPtr(), request.c_str(), -1, &prepared_statement, &unusedStatement);
	success &= (errorCode == SQLITE_OK);
	if(success)
	{
		cols = sqlite3_column_count(prepared_statement);
		// execute the statement
		do{
			errorCode = sqlite3_step(prepared_statement);
			switch( errorCode )
			{
			case SQLITE_DONE:
				break;
			case SQLITE_ROW:
				channel = loadChannel(cols , prepared_statement);
				break;
			default:
				break;
			}
		}while( errorCode == SQLITE_ROW );
	}
	else
	{
		ROS_ERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}

	errorCode = sqlite3_finalize(prepared_statement);
	return channel;
}

ChannelObjPtr ChannelDao::findByDescription(std::string description)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	ChannelObjPtr channel;

	buf << "SELECT * FROM channel WHERE description='" << description <<"'" ;
	request = buf.str();
	const char* unusedStatement;
	errorCode = sqlite3_prepare_v2(getDbPtr(), request.c_str(), -1, &prepared_statement, &unusedStatement);
	success &= (errorCode == SQLITE_OK);
	if(success)
	{
		cols = sqlite3_column_count(prepared_statement);
		// execute the statement
		do{
			errorCode = sqlite3_step(prepared_statement);
			switch( errorCode )
			{
			case SQLITE_DONE:
				break;
			case SQLITE_ROW:
				channel = channel = loadChannel(cols , prepared_statement);
				break;
			default:
				break;
			}
		}while( errorCode == SQLITE_ROW );
	}
	else
	{
		ROS_ERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}

	errorCode = sqlite3_finalize(prepared_statement);
	return channel;
}

std::map<int,ChannelARTptr> ChannelDao::findAll(int layerId)
{
	bool success = true;
	std::string request;
	std::ostringstream buf;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	std::map<int,ChannelARTptr> channelVector;
	ChannelObjPtr channel;

	buf << "SELECT * FROM channel WHERE layerId = "<< layerId << " ORDER BY layerId,indexChannel";
	request = buf.str();
	const char* unusedStatement;
	errorCode = sqlite3_prepare_v2(getDbPtr(), request.c_str(), -1, &prepared_statement, &unusedStatement);
	success &= (errorCode == SQLITE_OK);
	if(success)
	{
		cols = sqlite3_column_count(prepared_statement);
		// execute the statement
		do{
			errorCode = sqlite3_step(prepared_statement);
			switch( errorCode )
			{
			case SQLITE_DONE:
				break;
			case SQLITE_ROW:
				channel = loadChannel(cols , prepared_statement);
				channelVector[channel->getUid()] = ChannelARTptr(new ChannelART());
				channelVector[channel->getUid()]->setChannelObj(channel);
				break;
			default:
				break;
			}
		}while( errorCode == SQLITE_ROW );
	}
	else
	{
		ROS_ERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}

	errorCode = sqlite3_finalize(prepared_statement);
	return channelVector;
}

bool ChannelDao::clearTable()
{
	return  this->truncate();
}
