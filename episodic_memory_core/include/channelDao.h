/*
 * ChannelDao.h
 *
 *  Created on: 2012-09-17
 *      Author: Frank
 */

#ifndef CHANNELDAO_H_
#define CHANNELDAO_H_

#include "dao.h"
#include "channelART.h"

class ChannelDao;
typedef boost::shared_ptr<ChannelDao> ChannelDaoPtr;

/**
 * This class is use to interface with a database and a mapped object channel
 */
class ChannelDao: public Dao<ChannelObj>
{
public:
	/**
	 * Constructor
	 * @param db: sqlite connection object
	 */
	ChannelDao(sqlite3*);
	/**
	 * Destructor
	 */
	virtual ~ChannelDao();
	/**
	 * This function is used to insert a new objet in the database
	 * @param obj: the channel object to insert
	 * @return true if operation succeed, false otherwise
	 */
	bool insertObj(ChannelObjPtr obj);
	/**
	 * This function deletes an object in the database, IT IS NOT IMPLEMENTED
	 * @param obj: the channel object to delete
	 * @return false
	 */
	bool deleteObj(ChannelObjPtr obj);
	/**
	 * This function update an existing object. First it search if the object exist, then it updates
	 * @param obj: the channel object to update
	 * @return true if operation succeed, false otherwise
	 */
	bool updateObj(ChannelObjPtr obj);

	/**
	 * This function search a Input object specified by an id
	 * @param id: the identifiant of the channel object
	 * @return a shared pointer to the retrieved Input object
	 */
	ChannelObjPtr find(int uid);
	ChannelObjPtr findByDescription(std::string description);
	/**
	 * This function selects all the channel object
	 * @param id: not used
	 * @return : a vector that contains all the channel shared pointers
	 */
	std::map<int,ChannelARTptr> findAll(int layerId);
	/**
	 * This function clears the data in the table, leaving the column definition
	 * @return true if operation succeed, false otherwise
	 */
	bool clearTable();
};

#endif /* CHANNELDAO_H_ */
