/*
 * stmDesires.cpp
 *
 *  Created on: 2013-05-07
 *      Author: frank
 */
#include "stmDesires.h"

StmDesires::StmDesires(EgosPtr egos, ros::NodeHandle* nh, int duration):StmInterface(egos)
{
	eventGoToFilter_ = new iw::EventsFilter(*nh,"goto_EM_scenario");
	eventGoToFilter_->allEventsCB(&StmDesires::gotoAllEventCallBack, this);

	eventWanderFilter_ = new iw::EventsFilter(*nh,"wander_EM_scenario");
	eventWanderFilter_->allEventsCB(&StmDesires::wanderAllEventCallBack, this);

	eventTeleopFilter_ = new iw::EventsFilter(*nh,"irl1_teleop");
	eventTeleopFilter_->allEventsCB(&StmDesires::teleopAllEventCallBack, this);

	eventGraspFilter_ = new iw::EventsFilter(*nh,"object_grasp");
	eventGraspFilter_->allEventsCB(&StmDesires::graspAllEventCallBack, this);

	eventDeliverFilter_ = new iw::EventsFilter(*nh,"object_deliver");
	eventDeliverFilter_->allEventsCB(&StmDesires::deliverAllEventCallBack, this);

	setDuration(duration);

	//timer_ = nh->createTimer(ros::Duration(1),
	//		&StmDesires::timerCB, this);
}

void StmDesires::addDesireToEgosphere(std::string desire)
{
	Info::Itr desiresItr = egos_->infoByTag("exploited_desires");
	Params param;

	param = Params().value("id",desire);

	if(!desiresItr)
	{
		Info::Ptr desiresPtr = egos_->newInfo( "exploited_desires",  param);
		desiresPtr->cacheLimits(Info::Duration(getDuration()),3);
		setStmChanged(true);
		ROS_INFO("first desires in egosphere: %s",desire.c_str());
	}
	else
	{
		bool isPresent = false;
		//verify if the object id exist in the egosphere
		for(Info::Itr p = desiresItr; p; p++)
		{
			std::string egoDesires = p->get<std::string>("id");
			if(egoDesires.compare(desire) == 0)
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
			Info::Ptr infoDesires = egos_->newInfo("exploited_desires", param);
			infoDesires->cacheLimits(Info::Duration(getDuration()),10);
			setStmChanged(true);
			ROS_INFO("stm changed, new desire");
		}
		else
		{
			setStmChanged(false);
		}
	}
}

void StmDesires::expireDesire(std::string desire)
{
	for(Info::Itr p = egos_->infoByTag("exploited_desires"); p; p++)
	{
		try{
			std::string desireEgos = p->get<std::string>("id");
			ros::Time timestamp = p->getTimeStamp("id");
			int delay = ros::Time::now().sec - timestamp.sec;
			if(desireEgos.compare(desire) == 0 && delay >= getDuration())
			{
				p->expire();
			}
		}catch(egosphere::ParameterNotDefined &e){
			ROS_WARN("parameter id not defined stm gotoAllEvent");
		}
	}
}

void StmDesires::gotoAllEventCallBack(const hbba_msgs::Event& msg)
{
	ROS_DEBUG("stm goto event %i",msg.type);

	std::string desire = "Goto"; //maybe change with msg.type

	if(msg.type == hbba_msgs::Event::EXP_ON )
	{
		addDesireToEgosphere(desire);

		/*exploitedDesires["Goto"] = true;
		//it is impossible to have those desires at the same time
		exploitedDesires["Wander"] = false;*/
	}
	else if(msg.type == hbba_msgs::Event::EXP_OFF )
	{
		//expire the goto
		expireDesire(desire);

		//exploitedDesires["Goto"] = false;
	}
}

void StmDesires::teleopAllEventCallBack(const hbba_msgs::Event& msg)
{
	ROS_INFO("stm teleop event %i",msg.type);
	std::string desire = "Teleop";

	if(msg.type == hbba_msgs::Event::EXP_ON )
	{
		//exploitedDesires["Teleop"] = true;
		addDesireToEgosphere(desire);
	}
	else if(msg.type == hbba_msgs::Event::EXP_OFF )
	{
		//exploitedDesires["Teleop"] = false;
		//expire the teleop
		expireDesire(desire);
	}
}

void StmDesires::wanderAllEventCallBack(const hbba_msgs::Event& msg)
{
	ROS_DEBUG("stm wander event %i",msg.type);
	std::string desire = "Wander";

	if(msg.type == hbba_msgs::Event::EXP_ON )
	{
		addDesireToEgosphere(desire);
		//exploitedDesires["Wander"] = true;
		//it is impossible to have those desires at the same time
		//exploitedDesires["Goto"] = false;
	}
	else if(msg.type == hbba_msgs::Event::EXP_OFF )
	{
		//exploitedDesires["Wander"] = false;
		//expire the wander
		expireDesire(desire);
	}
}

void StmDesires::graspAllEventCallBack(const hbba_msgs::Event& msg)
{
	ROS_DEBUG("stm grasp event %i",msg.type);
	std::string desire = "grasp";

	if(msg.type == hbba_msgs::Event::EXP_ON )
	{
		//exploitedDesires["grasp"] = true;
		//it is impossible to have those desires at the same time
		//exploitedDesires["deliver"] = false;

		//find the object and associate it
		ros::Time now = ros::Time::now();
		int delay = 100;
		addDesireToEgosphere(desire);

		try{
			for(Info::Itr p = egos_->infoByTag("object"); p; p++)
			{
				ros::Time timestamp = p->getTimeStamp("objectId");
		
				if((now.sec - timestamp.sec ) < delay)
				{
					delay = now.sec - timestamp.sec;
					ROS_INFO("associate object and new grasp");
                        		for(Info::Itr d = egos_->infoByTag("exploited_desires"); d; d++)
                        		{
                                		if(d->get<std::string>("id").compare(desire) == 0)
                               		 	{
                                        		egos_->associate(d, p);
                                        		break;
                                		}
                        		}
				}
			}

		}catch(egosphere::ParameterNotDefined & e)
		{
			ROS_WARN("parameter not defined when associating object and graps in stm");
		}

	}
	else if(msg.type == hbba_msgs::Event::EXP_OFF )
	{
		//exploitedDesires["grasp"] = false;
		//grasp will be active until deliver occurs
	}

}

void StmDesires::deliverAllEventCallBack(const hbba_msgs::Event& msg)
{
	ROS_DEBUG("stm deliver event %i",msg.type);
	std::string desire = "deliver";

	if(msg.type == hbba_msgs::Event::EXP_ON )
	{
		//exploitedDesires["grasp"] = false;
		//it is impossible to have those desires at the same time
		//exploitedDesires["deliver"] = true;

		//dissociate object and grasp, add deliver to egosphere
		addDesireToEgosphere(desire);
		Info::Itr graspDesire = egos_->infoItr();
		for(Info::Itr p = egos_->infoByTag("exploited_desires"); p; p++)
		{
			if(p->get<std::string>("id").compare("grasp") == 0)
			{
				graspDesire = p;
				break;
			}
		}
		Info::Itr object = egos_->infoByAssociation(graspDesire,"object");
		if(object && graspDesire)
		{
			egos_->dissociate(object, graspDesire);
			ROS_INFO("dissociate object and grasp desire");
		}
		else{
			ROS_WARN("cant' dissociate object and grasp");
		}

	}
	else if(msg.type == hbba_msgs::Event::EXP_OFF )
	{
		//exploitedDesires["deliver"] = false;
		//expire graps and deliver
		expireDesire(desire);
		expireDesire("grasp");
	}
}
