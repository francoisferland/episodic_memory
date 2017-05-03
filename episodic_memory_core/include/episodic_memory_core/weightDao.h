/*
 * WeightDao.h
 *
 *  Created on: 2012-09-12
 *      Author: Frank
 */

#ifndef WEIGHTDAO_H_
#define WEIGHTDAO_H_

#include "dao.h"
#include "weightObj.h"
//#include <utilite/UTimer.h>

class WeightDao;
typedef boost::shared_ptr<WeightDao> WeightDaoPtr;

/**
 * This class is use to interface with a database and a mapped object Weight
 */
class WeightDao : public Dao<WeightObj>{
public:
	/**
	 * Constructor
	 * @param db: sqlite connection object
	 */
	WeightDao(sqlite3* db);
	/**
	 * Destructor
	 */
	virtual ~WeightDao();
	/**
	 * This function is used to insert a new objet in the database
	 * @param obj: the Weight object to insert
	 * @return true if operation succeed, false otherwise
	 */
	bool insertObj(WeightObjPtr obj);
	/**
	 * This function deletes an object in the database, IT IS NOT IMPLEMENTED
	 * @param obj: the Weight object to delete
	 * @return false
	 */
	bool deleteObj(WeightObjPtr obj);
	/**
	 * This function update an existing object. First it search if the object exist, then it updates
	 * @param obj: the Weight object to update
	 * @return true if operation succeed, false otherwise
	 */
	bool updateObj(WeightObjPtr obj);
	/**
	 * This function clears the data in the table, leaving the column definition
	 * @return true if operation succeed, false otherwise
	 */
	bool clearTable();
	/**
	 * This function search a Weight object specified by an id, IT IS NOT IMPLEMENTED
	 * @param id: the identifiant of the Weight object
	 * @return a shared pointer to the retrieved Weight object
	 */
	WeightObjPtr find(int uid);
	/**
	 * This function selects all the weight object in the same layer ART
	 * @param layer: the layer number
	 * @return : a vector that contains all the Weight shared pointers
	 */

	std::map<int,WeightObjPtr> findAll(int layer, std::map<int,CategoryARTptr> mapDownCategory, std::map<int,CategoryARTptr> mapUpCategory);
	WeightObjPtr findByConnection(CategoryARTptr downCategoryUid,CategoryARTptr upCategoryUid);

};

#endif /* WEIGHTDAO_H_ */
