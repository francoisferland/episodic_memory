/*
 * stmLocation.cpp
 *
 *  Created on: 2013-03-28
 *      Author: frank
 */
#include "stmLocation.h"

StmLocation::StmLocation(EgosPtr egos, int duration):StmInterface(egos)
{
	egos_ = egos;
	setDuration(duration);
}

void StmLocation::updateData(const perception_context_msgs::locationDetectionConstPtr location)
{
	ROS_INFO("current location %s , section = %s", location->location_label.c_str(), location->nearestSection.c_str());

	//verify if we have a location descriptor inside the egosphere
	Info::Itr locationItr = egos_->infoByTag("location");
	Params param;
	if(location->center_x != -1 && location->center_y != -1)
	{
		if(!location->nearestSection.empty())
		{
			param = Params().value("room",location->location_label)
				.value("center_x",location->center_x)
				.value("center_y",location->center_y)
				.value("section",location->nearestSection);
		}
		else
		{
			param = Params().value("room",location->location_label)
				.value("center_x",location->center_x)
				.value("center_y",location->center_y);
		}
	}
	else
	{
		if(!location->nearestSection.empty())
		{
			param = Params().value("room",location->location_label)
				.value("section",location->nearestSection);
		}
		else
		{
			param = Params().value("room",location->location_label);
		}
	}

	
	if(!locationItr)
	{
		Info::Ptr locationPtr = egos_->newInfo( "location",  param);
		locationPtr->cacheLimits(Info::Duration(getDuration()),1);
		setStmChanged(true);
	}
	else
	{
		//verify if the location has changed
		std::string lastLocation;
		try
		{
			lastLocation = locationItr->get<std::string>("room");
		}
		catch(egosphere::ParameterNotDefined& e)
		{
			ROS_WARN("param not defined");
		}

		locationItr->set( param );
		locationItr->cacheLimits(Info::Duration(getDuration()),1);
		if( location->location_label.compare(lastLocation) != 0 )
		{
			setStmChanged(true);
		}
	}
}
