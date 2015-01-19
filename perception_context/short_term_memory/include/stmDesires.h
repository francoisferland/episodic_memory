/*
 * stmPeople.cpp
 *
 *  Created on: 2013-03-28
 *      Author: frank
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iw/events_filter.hpp>
#include "stmInterface.hpp"

using namespace egosphere_ros;

typedef typename boost::shared_ptr<Egosphere> EgosPtr;

class StmDesires : public StmInterface
{
public:
	StmDesires(EgosPtr, ros::NodeHandle*, int duration);
	virtual ~StmDesires(){;}
	//void updateData();
	void addDesireToEgosphere(std::string desire);

private:
	void gotoAllEventCallBack(const hbba_msgs::Event& msg);
	void wanderAllEventCallBack(const hbba_msgs::Event& msg);
	void teleopAllEventCallBack(const hbba_msgs::Event& msg);
	void graspAllEventCallBack(const hbba_msgs::Event& msg);
	void deliverAllEventCallBack(const hbba_msgs::Event& msg);
	//void timerCB(const ros::TimerEvent&);
	void expireDesire(std::string desire);

	iw::EventsFilter * eventGoToFilter_;
	iw::EventsFilter * eventWanderFilter_;
	iw::EventsFilter * eventTeleopFilter_;
	iw::EventsFilter * eventGraspFilter_;
	iw::EventsFilter * eventDeliverFilter_;

	std::map<std::string,bool> exploitedDesires;

	//ros::Timer timer_;

};

