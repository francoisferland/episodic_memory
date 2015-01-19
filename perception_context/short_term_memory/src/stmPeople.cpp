/*
 * stmPeople.cpp
 *
 *  Created on: 2013-03-28
 *      Author: frank
 */

#include "stmPeople.h"

StmPeople::StmPeople(EgosPtr egos, int duration):StmInterface(egos)
{
	egos_ = egos;
	setDuration(duration);
}

void StmPeople::updateData(const std_msgs::StringConstPtr person)
{
	ROS_DEBUG("%s identified in the context, stm",person->data.c_str());

	Info::Itr personItr = egos_->infoByTag("person");
	Params param = Params().value("name",person->data);

	if(!personItr)
	{
		Info::Ptr infoPerson = egos_->newInfo("person", param);
		infoPerson->cacheLimits(Info::Duration(getDuration()),10);
		setStmChanged(true);
	}
	else
	{
		bool isPresent = false;
		//verify if the object id exist in the egosphere
		for(Info::Itr p = personItr; p; p++)
		{
			std::string egoPerson = p->get<std::string>("name");
			if(egoPerson.compare(person->data) == 0)
			{
				//update the cache duration
				p->set( param );
				p->cacheDurationLimit(Info::Duration(getDuration()));
				isPresent = true;
			}
		}
		if(!isPresent)
		{
			//add the object to the egosphere
			Info::Ptr infoObject = egos_->newInfo("person", param);
			infoObject->cacheLimits(Info::Duration(getDuration()),10);
			setStmChanged(true);
		}
	}
}

