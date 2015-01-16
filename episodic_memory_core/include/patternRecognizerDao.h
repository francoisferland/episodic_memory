
#ifndef PATTERNRECOGNIZERDAO_H_
#define PATTERNRECOGNIZERDAO_H_

#include "dao.h"
#include "patternRecognizerObj.h"


class PatternRecognizerDao;
typedef boost::shared_ptr<PatternRecognizerDao> PatternRecognizerDaoPtr;

/**
 * This class is use to interface with a database and a mapped object Input
 */
class PatternRecognizerDao: public Dao<PatternRecognizerObj > {
public:
	/**
	 * Constructor
	 * @param db: sqlite connection object
	 */
	PatternRecognizerDao(sqlite3*);
	/**
	 * Destructor
	 */
	virtual ~PatternRecognizerDao(){}
	/**
	 * This function is used to insert a new objet in the database
	 * @param obj: the Input object to insert
	 * @return true if operation succeed, false otherwise
	 */
	bool insertObj(PatternRecognizerObjPtr obj);
	/**
	 * This function deletes an object in the database, IT IS NOT IMPLEMENTED
	 * @param obj: the Input object to delete
	 * @return false
	 */
	bool deleteObj(PatternRecognizerObjPtr obj);
	/**
	 * This function update an existing object. First it search if the object exist, then it updates
	 * @param obj: the Input object to update
	 * @return true if operation succeed, false otherwise
	 */
	bool updateObj(PatternRecognizerObjPtr obj);
	/**
	 * This function search a Input object specified by an id
	 * @param id: the identifiant of the Input object
	 * @return a shared pointer to the retrieved Input object
	 */
	PatternRecognizerObjPtr find(int uid);
	/**
	 * This function selects all the input object
	 * @return : a vector that contains all the Input shared pointers
	 */
	std::vector<PatternRecognizerObjPtr> findAll(int layerId);
	std::map<int,CategoryARTptr> findAllByChannel(boost::shared_ptr<ChannelART> channel);
	/**
	 * This function clears the data in the table, leaving the column definition
	 * @return true if operation succeed, false otherwise
	 */
	bool clearTable();
};

#endif /* PATTERNRECOGNIZERDAO_H_ */
