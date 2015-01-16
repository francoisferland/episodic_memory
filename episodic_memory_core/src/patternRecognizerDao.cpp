
#include <patternRecognizerDao.h>
#include "channelART.h"

PatternRecognizerDao::PatternRecognizerDao(sqlite3* db) : Dao<PatternRecognizerObj>(db,"patternRecognizer")
{
}

bool PatternRecognizerDao::insertObj(PatternRecognizerObjPtr obj)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;

	buf << "INSERT INTO patternRecognizer (indexPatternRecognizer, fk_channel, vigilance, learningRate) "
			<<"VALUES (" << obj->getIndexCategory() << ", " << obj->getChannel()->getUid() << ", "
			<< obj->getVigilance() << ", "<< obj->getLearningRate() <<")";

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
		UERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		UERROR("The query is: %s",request.c_str());
	}

	return success;
}

bool PatternRecognizerDao::deleteObj(PatternRecognizerObjPtr obj)
{
	return false;
}

bool PatternRecognizerDao::updateObj(PatternRecognizerObjPtr obj)
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
		buf << "UPDATE patternRecognizer SET vigilance='" << obj->getVigilance() << "', learningRate=" << obj->getLearningRate() <<
				", indexPatternRecognizer=" << obj->getIndexCategory() <<
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
			UERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
			UERROR("The query is: %s",request.c_str());
		}
	}

	return success;
}

PatternRecognizerObjPtr loadPatternRecognizer(int cols , sqlite3_stmt* prepared_statement)
{
	PatternRecognizerObjPtr patternRecognizer = PatternRecognizerObjPtr(new PatternRecognizerObj());
	for( int col = 0; col < cols; col++)
	{
		std::string columnName = sqlite3_column_name(prepared_statement, col);
		if(columnName.compare("indexPatternRecognizer") == 0)
		{
			patternRecognizer->setIndexCategory((int)sqlite3_column_int(prepared_statement, col));
		}
		else if(columnName.compare("vigilance") == 0)
		{
			patternRecognizer->setVigilance((float)sqlite3_column_double(prepared_statement, col));
		}
		else if(columnName.compare("learningRate") == 0)
		{
			patternRecognizer->setLearningRate((float)sqlite3_column_double(prepared_statement, col));
		}
		else if(columnName.compare("fk_channel") == 0)
		{
			//input->setChannel((int)sqlite3_column_int(prepared_statement, col));
		}
		else if(columnName.compare("uid") == 0)
		{
			patternRecognizer->setUID((int)sqlite3_column_int(prepared_statement,col));
		}
	}

	return patternRecognizer;
}

PatternRecognizerObjPtr PatternRecognizerDao::find(int inputId)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	PatternRecognizerObjPtr patternRecognizer;

	buf << "SELECT uid, indexPatternRecognizer, fk_channel, vigilance, learningRate"<<
			" FROM patternRecognizer WHERE patternRecognizer.uid=" << inputId ;
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
				patternRecognizer = loadPatternRecognizer(cols , prepared_statement);
				break;
			default:
				break;
			}
		}while( errorCode == SQLITE_ROW );
	}
	else
	{
		UERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		UERROR("The query is: %s",request.c_str());

	}

	errorCode = sqlite3_finalize(prepared_statement);
	return patternRecognizer;
}

std::vector<PatternRecognizerObjPtr> PatternRecognizerDao::findAll(int layerId)
{
	bool success = true;
	std::string request;
	std::ostringstream buf;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	std::vector<PatternRecognizerObjPtr> patternRecognizerVector;
	PatternRecognizerObjPtr patternRecognizer;

	buf <<  "SELECT patternRecognizer.uid, patternRecognizer.indexPatternRecognizer, patternRecognizer.fk_channel, patternRecognizer.vigilance, patternRecognizer.learningRate FROM patternRecognizer JOIN channel ON patternRecognizer.fk_channel = channel.uid WHERE channel.layerId = "<< layerId << " ORDER BY patternRecognizer.indexPatternRecognizer";
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
				patternRecognizer = loadPatternRecognizer(cols , prepared_statement);
				patternRecognizerVector.push_back(patternRecognizer);
				break;
			default:
				break;
			}
		}while( errorCode == SQLITE_ROW );
	}
	else
	{
		UERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		UERROR("The query is: %s",request.c_str());

	}

	errorCode = sqlite3_finalize(prepared_statement);
	return patternRecognizerVector;
}

std::map<int,CategoryARTptr> PatternRecognizerDao::findAllByChannel(boost::shared_ptr<ChannelART> channel)
{
	bool success = true;
	std::string request;
	std::ostringstream buf;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	std::map<int,CategoryARTptr>  PatternRecognizerVector;
	PatternRecognizerObjPtr patternRecognizer;

	buf << "SELECT patternRecognizer.uid, patternRecognizer.indexPatternRecognizer, patternRecognizer.fk_channel, patternRecognizer.vigilance, patternRecognizer.learningRate FROM patternRecognizer JOIN channel ON patternRecognizer.fk_channel = channel.uid WHERE channel.uid = "<< channel->getUid() <<" ORDER BY patternRecognizer.indexPatternRecognizer";
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
				patternRecognizer = loadPatternRecognizer(cols , prepared_statement);
				patternRecognizer->setChannel(channel);
				PatternRecognizerVector[patternRecognizer->getUid()] = patternRecognizer;
				break;
			default:
				break;
			}
		}while( errorCode == SQLITE_ROW );
	}
	else
	{
		UERROR("DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		UERROR("The query is: %s",request.c_str());

	}

	errorCode = sqlite3_finalize(prepared_statement);
	return PatternRecognizerVector;
}

bool PatternRecognizerDao::clearTable()
{
	return  this->truncate();
}
