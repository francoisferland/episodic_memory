/*
 * episodic_memory_planner.h
 *
 *  Created on: 2013-05-29
 *      Author: frank
 */

#ifndef EPISODIC_MEMORY_PLANNER_H_
#define EPISODIC_MEMORY_PLANNER_H_

#include <boost/function.hpp>
#include <ros/ros.h>
#include <ros_episodic_memory/recalledEpisode.h>
#include <ros_episodic_memory/contextRepresentation.h>

using namespace std;

namespace ros_episodic_memory
{

class EpisodicMemoryPlanner
{
public:
	EpisodicMemoryPlanner(ros::NodeHandle& n, ros::NodeHandle& np);
	~EpisodicMemoryPlanner(){;}

	void recalledEpisodeCB(const ros_episodic_memory::recalledEpisode::ConstPtr& msg );

	void addTriggerItem(std::string item);
	void removeTriggerItem(std::string item);

	template <class T>
	void relevantEventsCB(void (T::*fun)(const std::vector<ros_episodic_memory::contextRepresentation>&), T* obj)
	{
		relevantEventCB_ = boost::bind(fun, obj, _1);
	}

private:
	ros::Subscriber subRecalledEvent_;
	int nb_anticipated_event_;
	std::set<std::string> watchedItem;

	boost::function<void (const std::vector<ros_episodic_memory::contextRepresentation>&)> relevantEventCB_;
};

#endif /* EPISODIC_MEMORY_PLANNER_H_ */
}
