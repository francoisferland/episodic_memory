/*
 * DaoFactory.h
 *
 *  Created on: 2012-09-12
 *      Author: Frank
 */

#ifndef DAOFACTORY_H_
#define DAOFACTORY_H_

#include "dao.h"
#include "weightDao.h"
#include "inputDao.h"
#include "channelDao.h"
#include "patternRecognizerDao.h"

/**
 * This class centralize the creations of all the dao object
 */
class DaoFactory {
public:
	/**
	 * constructor
	 */
	DaoFactory();
	/**
	 * Destructor
	 */
	virtual ~DaoFactory();
	/**
	 * Getter function, weightDao object
	 * @param pointer to sqlite connection object
	 */
	static WeightDaoPtr getWeightDao(sqlite3*);
	/**
	 * Getter function, inputDao object
	 * @param pointer to sqlite connection object
	 */
	static InputDaoPtr getInputDao(sqlite3*);
	/**
	 * Getter function, channelDao object
	 * @param pointer to sqlite connection object
	 */
	static ChannelDaoPtr getChannelDao(sqlite3*);

	/**
	 * Getter function, channelDao object
	 * @param pointer to sqlite connection object
	 */
	static PatternRecognizerDaoPtr getPatternRecognizerDao(sqlite3*);

};

#endif /* DAOFACTORY_H_ */
