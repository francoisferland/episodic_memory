/*
 * stmPeople.cpp
 *
 *  Created on: 2013-03-28
 *      Author: frank
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "stmInterface.hpp"

class StmPeople : public StmInterface
{
public:
	StmPeople(EgosPtr,int duration);
	~StmPeople(){;}
	void updateData(const std_msgs::StringConstPtr person);


};

