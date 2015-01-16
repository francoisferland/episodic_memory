/*
 * InputDao.h
 *
 *  Created on: 2012-09-17
 *      Author: Frank
 */

#ifndef INPUTDAO_H_
#define INPUTDAO_H_

#include "dao.h"
#include "inputObj.h"


class InputDao;
typedef boost::shared_ptr<InputDao> InputDaoPtr;

/**
 * This class is use to interface with a database and a mapped object Input
 */
class InputDao: public Dao<InputObj > {
public:
	/**
	 * Constructor
	 * @param db: sqlite connection object
	 */
	InputDao(sqlite3*);
	/**
	 * Destructor
	 */
	virtual ~InputDao();
	/**
	 * This function is used to insert a new objet in the database
	 * @param obj: the Input object to insert
	 * @return true if operation succeed, false otherwise
	 */
	bool insertObj(InputObjPtr obj);
	/**
	 * This function deletes an object in the database, IT IS NOT IMPLEMENTED
	 * @param obj: the Input object to delete
	 * @return false
	 */
	bool deleteObj(InputObjPtr obj);
	/**
	 * This function update an existing object. First it search if the object exist, then it updates
	 * @param obj: the Input object to update
	 * @return true if operation succeed, false otherwise
	 */
	bool updateObj(InputObjPtr obj);
	/**
	 * This function search a Input object specified by an id
	 * @param id: the identifiant of the Input object
	 * @return a shared pointer to the retrieved Input object
	 */
	InputObjPtr find(int uid);
	/**
	 * This function search a Input object specified by its description
	 * @param description
	 * @return a shared pointer to the retrieved Input object
	 */
	InputObjPtr findByDescription(std::string description);

	std::map<int,boost::shared_ptr<CategoryART> > findAllByChannel(boost::shared_ptr<ChannelART> channel);
	/**
	 * This function selects all the input object
	 * @return : a vector that contains all the Input shared pointers
	 */
	std::map<int,CategoryARTptr> findAll();
	/**
	 * This function clears the data in the table, leaving the column definition
	 * @return true if operation succeed, false otherwise
	 */
	bool clearTable();
};

#endif /* INPUTDAO_H_ */
