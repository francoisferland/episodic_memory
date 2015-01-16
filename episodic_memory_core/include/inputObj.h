/*
 * InputObj.h
 *
 *  Created on: 2012-12-12
 *      Author: frank
 */

#ifndef INPUTOBJ_H_
#define INPUTOBJ_H_

#include <iostream>
#include <boost/thread.hpp>
#include "categoryART.h"

class InputObj;
typedef boost::shared_ptr<InputObj> InputObjPtr;

/**
 * This class represent an object of the Input Table in the database
 */
class InputObj : public CategoryART
{
public:
	/**
	 * default Constructor
	 */
	InputObj();
	/**
	 * Constructor
	 * @param number: id number in the layer
	 * @param channel: channel id
	 * @param desc: a textual description of the input
	 */
	InputObj(int uid, int indexInput, std::string desc);
	/**
	 * Destructor
	 */
	virtual ~InputObj(){}
	/**
	 * Getter function, description
	 * @return description
	 */
	std::string getDescription() const
	{
		return m_description;
	}
	/**
	 * Setter function, description
	 */
	void setDescription(std::string description)
	{
		this->m_description = description;
	}

	InputObjPtr getComplementInput() const {
		return m_complementInput;
	}

	void setComplementInput(const InputObjPtr complementInput) {
		m_complementInput = complementInput;
	}

private:

	std::string m_description;
	InputObjPtr m_complementInput;

};

namespace EM_CORE
{
class InputROSmsg;
typedef boost::shared_ptr<InputROSmsg> InputROSmsgPtr;

class InputROSmsg
{
public:
	InputROSmsg(std::string description,std::string channelDescription,float activationValue,int matchRelevance)
	{description_=description;channelDescription_=channelDescription;activationValue_=activationValue;matchRelevance_=matchRelevance;}

	std::string getDesc(){return description_;}
	std::string getChannelDesc(){return channelDescription_;}
	float getActivValue(){return activationValue_;}
	int getMatchRelev(){return matchRelevance_;}

private:
	std::string description_;
	std::string channelDescription_;
	float activationValue_;
	int matchRelevance_;

};
}


#endif /* INPUTOBJ_H_ */
