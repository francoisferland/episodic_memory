/*
 * stmInterface.h
 *
 *  Created on: 2013-05-07
 *      Author: frank
 */

#ifndef STMINTERFACE_H_
#define STMINTERFACE_H_

#include "egosphere_ros/egosphere.hpp"

using namespace egosphere_ros;
typedef typename boost::shared_ptr<Egosphere> EgosPtr;

class StmInterface
{

public:

	StmInterface(EgosPtr egos):stmChanged_(false),duration_(0){egos_ = egos;}
	virtual ~StmInterface(){;}

	int getDuration() const {
		return duration_;
	}
	void setDuration(int duration)
	{
		duration_ = duration;
	}

	bool isStmChanged() const {
		return stmChanged_;
	}

	void setStmChanged(bool stmChanged) {
		this->stmChanged_ = stmChanged;
	}

protected:
	//protected variable
	EgosPtr egos_;

private:
	bool stmChanged_;
	int duration_;


};


#endif /* STMINTERFACE_H_ */
