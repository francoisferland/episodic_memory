/*
 * episodic_memory_planner.cpp
 *
 *  Created on: 2013-05-29
 *      Author: frank
 */

#include <episodic_memory_planner.hpp>

using namespace ros_episodic_memory;

EpisodicMemoryPlanner::EpisodicMemoryPlanner(ros::NodeHandle& n, ros::NodeHandle& np)
{
	subRecalledEvent_ = n.subscribe("recalled_episode", 25,
			&EpisodicMemoryPlanner::recalledEpisodeCB, this);

	np.param("nb_anticipated_events",nb_anticipated_event_,1);

}

void EpisodicMemoryPlanner::recalledEpisodeCB(const ros_episodic_memory::recalledEpisode::ConstPtr& msg )
{
	std::vector<ros_episodic_memory::contextRepresentation> listRelevantEvent;
	//verify if the episode contains relevant information
	for(unsigned int i = 0 ; i < msg->realIndex.size() ; i++)
	{
		ROS_INFO("em planner : event %i",msg->realIndex[i]);
		//look into the anticipated event
		//if an anticipated event contain a watched item, store the associated event
		for(unsigned int j = 0 ; j < msg->listInput[i].listInputDescription.size() ; j++ )
		{
//			ROS_INFO("%s, %s",msg->listInput[i].listInputDescription[j].c_str(),msg->listInput[i].listAssociatedChannel[j].c_str());
			if(watchedItem.count(msg->listInput[i].listInputDescription[j]))
			{
				listRelevantEvent.push_back(msg->listInput[i]);
				ROS_INFO("relevant info in memory: event %i",msg->realIndex[i]);
				break;
			}
		}
	}

	if(listRelevantEvent.size() > 0 && (int)listRelevantEvent.size() >= nb_anticipated_event_ )
	{
		relevantEventCB_(listRelevantEvent);
	}

	//clear episode
	listRelevantEvent.clear();
}

void EpisodicMemoryPlanner::addTriggerItem(std::string item)
{
	watchedItem.insert(item);
}

void EpisodicMemoryPlanner::removeTriggerItem(std::string item)
{
	if(watchedItem.count(item))
	{
		watchedItem.erase(item);
	}
}

