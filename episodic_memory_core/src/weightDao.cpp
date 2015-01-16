#include "weightDao.h"
#include "layerART.h"

WeightDao::WeightDao(sqlite3* db) : Dao<WeightObj>(db,"weight")
{
}

WeightDao::~WeightDao() {
}

bool WeightDao::insertObj(WeightObjPtr obj)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;

	//UTimer timer;
	buf << "INSERT INTO weight (fk_upCategory, fk_downCategory, weightValue) "
			<<"VALUES (" << obj->getUpCategoryPtr()->getIndexCategory() << ", " << obj->getDownCategoryPtr()->getIndexCategory() << ", " << obj->getValue() << ") ";
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
		ROS_ERROR("insertObj DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}

	//UDEBUG("Time for insert weight = %f s", timer.ticks());

	return success;
}

bool WeightDao::deleteObj(WeightObjPtr obj)
{
	return false;
}

bool WeightDao::updateObj(WeightObjPtr obj)
{
	bool success = true;
	bool found = false;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = -1;
	//try to find the object in the table before updating
	if( this->find(obj->getUID()) )
		found = true;
	else
		found = false;

	//UTimer timer;

	if(found)
	{
		buf << "UPDATE weight SET weightValue=" << obj->getValue() << " WHERE uid=" << obj->getUID();
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
			ROS_ERROR("updateObj DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
			ROS_ERROR("The query is: %s",request.c_str());
		}
	}

	//UDEBUG("Time for update weight = %f s", timer.ticks());

	return success;

}

WeightObjPtr loadWeight(int cols, sqlite3_stmt* prepared_statement,std::map<int,CategoryARTptr> mapDownCategory, std::map<int,CategoryARTptr> mapUpCategory)
{
	WeightObjPtr weight = WeightObjPtr(new WeightObj());
	for( int col = 0; col < cols; col++)
	{
		std::string columnName = sqlite3_column_name(prepared_statement, col);
		if(columnName.compare("fk_upCategory") == 0)
		{
			int indexUpCategory = (int)sqlite3_column_int(prepared_statement, col);
			for(std::map<int,CategoryARTptr>::iterator it = mapUpCategory.begin() ; it!= mapUpCategory.end() ; it++)
			{
				if((*it).second->getIndexCategory() == indexUpCategory)
				{
					weight->setUpCategoryPtr((*it).second);
					(*it).second->addDownWeight(weight);
					break;
				}
			}
		}
		else if(columnName.compare("fk_downCategory") == 0)
		{
			int indexDownCategory = (int)sqlite3_column_int(prepared_statement, col);
			for(std::map<int,CategoryARTptr>::iterator it = mapDownCategory.begin() ; it!= mapDownCategory.end() ; it++)
			{
				if((*it).second->getIndexCategory() == indexDownCategory)
				{
					weight->setDownCategoryPtr((*it).second);
					(*it).second->addUpWeight(weight);
					break;
				}
			}
		}
		else if(columnName.compare("uid") == 0)
		{
			weight->setUID((int)sqlite3_column_int(prepared_statement, col));
		}
		else if(columnName.compare("weightValue") == 0)
		{
			weight->setValue((double)sqlite3_column_double(prepared_statement, col));
		}
	}
	return weight;
}

WeightObjPtr loadWeight(int cols, sqlite3_stmt* prepared_statement)
{
	//empty maps
	std::map<int,CategoryARTptr> mapDownCategory;
	std::map<int,CategoryARTptr> mapUpCategory;
	return loadWeight(cols,prepared_statement,mapDownCategory,mapUpCategory);
}

WeightObjPtr WeightDao::find(int uid)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	WeightObjPtr weight;

	//UTimer timer;
	buf << "SELECT uid, fk_upCategory, fk_downCategory, weightValue FROM weight WHERE uid=" << uid ;
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
				weight = loadWeight(cols, prepared_statement);

				break;
			default:
				break;
			}
		}while( errorCode == SQLITE_ROW );
	}
	else
	{
		ROS_ERROR("find DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}

	errorCode = sqlite3_finalize(prepared_statement);

	//UDEBUG("Time for find weight = %f s", timer.ticks());

	return weight;
}

std::map<int,WeightObjPtr> WeightDao::findAll(int layer, std::map<int,CategoryARTptr> mapDownCategory, std::map<int,CategoryARTptr> mapUpCategory)
{
	std::map<int,WeightObjPtr> listWeight;

	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	WeightObjPtr weight;

	if(layer == INPUT_LAYER)
		buf << "SELECT W.uid, W.fk_upCategory, W.fk_downCategory, W.weightValue FROM weight AS W JOIN input             AS I ON W.fk_downCategory=I.indexInput             JOIN channel AS C ON I.fk_channel=C.uid WHERE C.layerId=" << layer ;
	else if(layer == EVENT_LAYER)
		buf << "SELECT W.uid, W.fk_upCategory, W.fk_downCategory, W.weightValue FROM weight AS W JOIN patternRecognizer AS I ON W.fk_downCategory=I.indexPatternRecognizer JOIN channel AS C ON I.fk_channel=C.uid WHERE C.layerId=" << layer ;

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
				weight = loadWeight(cols, prepared_statement,mapDownCategory,mapUpCategory);

				listWeight[weight->getUID()] = weight;
				break;
			default:
				break;
			}
		}while( errorCode == SQLITE_ROW );
	}
	else
	{
		ROS_ERROR("findAll DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}

	errorCode = sqlite3_finalize(prepared_statement);
	return listWeight;
}

WeightObjPtr WeightDao::findByConnection(CategoryARTptr downCategory,CategoryARTptr upCategory)
{
	bool success = true;
	std::ostringstream buf;
	std::string request;
	sqlite3_stmt* prepared_statement;
	int errorCode = 0;
	int cols = 0;
	WeightObjPtr weight;

	//UTimer timer;
	buf << "SELECT uid, fk_upCategory, fk_downCategory, weightValue FROM weight WHERE fk_downCategory=" << downCategory->getIndexCategory() << " AND fk_upCategory=" << upCategory->getIndexCategory()  ;
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
				weight = loadWeight(cols, prepared_statement);
				weight->setUpCategoryPtr(upCategory);
				weight->setDownCategoryPtr(downCategory);

				break;
			default:
				break;
			}
		}while( errorCode == SQLITE_ROW );
	}
	else
	{
		ROS_ERROR("find DB error (%d): %s", errorCode, sqlite3_errmsg(getDbPtr()));
		ROS_ERROR("The query is: %s",request.c_str());
	}

	errorCode = sqlite3_finalize(prepared_statement);

	//UDEBUG("Time for find weight = %f s", timer.ticks());

	return weight;
}

bool WeightDao::clearTable()
{
	return  this->truncate();
}
