#ifndef WEIGTHOBJ_H_
#define WEIGTHOBJ_H_

#include <boost/thread.hpp>
#include "categoryART.h"

class WeightObj;
typedef boost::shared_ptr<WeightObj> WeightObjPtr;

/**
 * This class represent an object of the Weight Table in the database
 */
class WeightObj {
public:
	/**
	 * Constructor
	 */
	WeightObj(): m_uid(-1),m_value(0),isUpdated_(true){;}
	/**
	 * Destructor
	 */
	virtual ~WeightObj(){;}
	/**
	 * Getter function, column
	 * @return column
	 */
	CategoryARTptr getDownCategoryPtr() const
    {
        return m_downCategoryPtr;
    }
	/**
	 * Setter function, column
	 */
    void setDownCategoryPtr(CategoryARTptr downCategoryPtr)
    {
        this->m_downCategoryPtr = downCategoryPtr;
    }
	/**
	 * Getter function, row
	 * @return row
	 */
    CategoryARTptr getUpCategoryPtr() const
    {
        return m_upCategoryPtr;
    }
	/**
	 * Setter function, row
	 */
    void setUpCategoryPtr(CategoryARTptr upCategoryPtr)
    {
        this->m_upCategoryPtr = upCategoryPtr;
    }
	/**
	 * Getter function, value
	 * @return column
	 */
    double getValue() const
    {
        return m_value;
    }
	/**
	 * Setter function, value
	 */
    void setValue(double value)
    {
    	if(std::abs(value - m_value) > 0.001)
    		isUpdated_ = true;
    	else
    		isUpdated_ = false;
        this->m_value = value;
    }

	int getUID() const {
		return m_uid;
	}

	void setUID(int uid) {
		this->m_uid = uid;
	}

	bool isConnected(CategoryARTptr category);

	bool isUpdated() const {
		return isUpdated_;
	}

	void setIsUpdated(bool isUpdated) {
		isUpdated_ = isUpdated;
	}

private:

    CategoryARTptr m_upCategoryPtr;
    CategoryARTptr m_downCategoryPtr;
    int m_uid;
    double m_value;
    bool isUpdated_;

};

#endif /* WEIGTHOBJ_H_ */
