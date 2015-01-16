/*
 * DaoFactory.cpp
 *
 *  Created on: 2012-09-12
 *      Author: Frank
 */

#include <daoFactory.h>

DaoFactory::DaoFactory() {

}

DaoFactory::~DaoFactory() {
}

WeightDaoPtr DaoFactory::getWeightDao(sqlite3* db)
{
	return WeightDaoPtr(new WeightDao(db));
}

InputDaoPtr DaoFactory::getInputDao(sqlite3* db)
{
	return InputDaoPtr(new InputDao(db));
}

ChannelDaoPtr DaoFactory::getChannelDao(sqlite3* db)
{
	return ChannelDaoPtr(new ChannelDao(db));
}

PatternRecognizerDaoPtr DaoFactory::getPatternRecognizerDao(sqlite3* db)
{
	return PatternRecognizerDaoPtr(new PatternRecognizerDao(db));
}
