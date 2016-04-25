/*
 * InputDao.cpp
 *
 *  Created on: 2012-09-17
 *      Author: Frank
 */

#include <inputDao.h>
#include <channelART.h>

InputDao::InputDao(sqlite3* db) : Dao<InputObj>(db,"input")
{
	bool success = true;
	sqlite3_stmt* prepared_statement;
    const char* unusedStatement;
	int errorCode = 0;

    std::string request =
        "create table if not exists input "
        "(uid INTEGER PRIMARY KEY,"
        " indexInput smallint,"
        " fk_channel smallint,"
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

InputDao::~InputDao()
{
}

bool InputDao::insertObj(InputObjPtr obj)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;

	buf << "INSERT INTO input (indexInput, fk_channel, description) "
			<<"VALUES (" << obj->getIndexCategory() << ", " << obj->getChannel()->getUid() << ", '" << obj->getDescription() << "')";

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
		obj->setUID(lastId);
	}

	if(!success)
	{
		ROS_ERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}

	return success;
}

bool InputDao::deleteObj(InputObjPtr obj)
{
	return false;
}

bool InputDao::updateObj(InputObjPtr obj)
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
		buf << "UPDATE input SET description='" << obj->getDescription() << "', fk_channel=" << obj->getChannel()->getUid() <<
				" WHERE uid=" << obj->getUid() ;
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

InputObjPtr loadInput(int cols , sqlite3_stmt* prepared_statement)
{
	InputObjPtr input = InputObjPtr(new InputObj());
	for( int col = 0; col < cols; col++)
	{
		std::string columnName = sqlite3_column_name(prepared_statement, col);
		if(columnName.compare("indexInput") == 0)
		{
			input->setIndexCategory((int)sqlite3_column_int(prepared_statement, col));
		}
		else if(columnName.compare("description") == 0)
		{
			input->setDescription((const char*)sqlite3_column_text(prepared_statement, col));
		}
		else if(columnName.compare("fk_channel") == 0)
		{
			//input->setChannel((int)sqlite3_column_int(prepared_statement, col));
		}
		else if(columnName.compare("uid") == 0)
		{
			input->setUID((int)sqlite3_column_int(prepared_statement,col));
		}

	}

	return input;
}

InputObjPtr InputDao::find(int inputId)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	InputObjPtr input;

	buf << "SELECT input.indexInput, input.description, input.fk_channel, input.uid FROM input JOIN channel ON input.fk_channel = channel.uid WHERE input.uid=" << inputId ;
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
				input = loadInput(cols , prepared_statement);
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
	return input;
}

InputObjPtr InputDao::findByDescription(std::string description)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	InputObjPtr input;

	buf << "SELECT indexInput, description, fk_channel, uid FROM input WHERE description='" << description <<"'" ;
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
				input = loadInput(cols , prepared_statement);
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
	return input;
}

std::map<int,CategoryARTptr> InputDao::findAll()
{
	bool success = true;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	std::map<int,CategoryARTptr> inputVector;
	InputObjPtr input;

	request =  "SELECT input.indexInput, input.description, input.fk_channel, input.uid FROM input JOIN channel ON input.fk_channel = channel.uid WHERE channel.layerId = 1 ORDER BY input.indexInput";
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
				input = loadInput(cols , prepared_statement);
				inputVector[input->getUid()] = input;
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
	return inputVector;
}

std::map<int,CategoryARTptr> InputDao::findAllByChannel(boost::shared_ptr<ChannelART> channel)
{
	bool success = true;
	std::string request;
	std::ostringstream buf;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	std::map<int,CategoryARTptr>  inputVector;
	InputObjPtr input;

	buf << "SELECT input.indexInput, input.description, input.fk_channel, input.uid FROM input JOIN channel ON input.fk_channel = channel.uid WHERE channel.uid = "<< channel->getUid() <<" ORDER BY input.indexInput";
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
				input = loadInput(cols , prepared_statement);
				input->setChannel(channel);
				inputVector[input->getUid()] = input;
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
	return inputVector;
}

bool InputDao::clearTable()
{
	return  this->truncate();
}
