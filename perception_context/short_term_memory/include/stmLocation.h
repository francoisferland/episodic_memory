/*
 * stmLocation.cpp
 *
 *  Created on: 2013-03-28
 *      Author: frank
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "perception_context_msgs/locationDetection.h"
#include "stmInterface.hpp"

class StmLocation: public StmInterface
{
public:
	StmLocation(EgosPtr, int duration);
	~StmLocation(){;}

	void updateData(const perception_context_msgs::locationDetectionConstPtr location);

};

