/*
 * stmEmotions.cpp
 *
 *  Created on: 2013-09-03
 *      Author: frank
 */
#include "stmEmotions.h"

StmEmotion::StmEmotion(EgosPtr egos, int duration):StmInterface(egos)
{
	egos_ = egos;
	setDuration(duration);
}

void StmEmotion::updateData(const hbba_msgs::EmotionIntensitiesConstPtr emotions)
{
	for(std::vector<emotions_msgs::Intensity>::const_iterator it = emotions->emotion.begin() ; it != emotions->emotion.end() ; it++)
	{
		ROS_DEBUG("%s (%5.2f) emotion in stm ", (*it).name.c_str() , (*it).value);

		Info::Itr emotionItr = egos_->infoByTag("emotion");
		Params param = Params().value("name",(*it).name)
								.value("intensity",(*it).value);

		if(!emotionItr)
		{
			Info::Ptr infoEmotion = egos_->newInfo("emotion", param);
			infoEmotion->cacheLimits(Info::Duration(getDuration()),10);
			setStmChanged(true);
		}
		else
		{
			bool isPresent = false;
			//verify if the emotion exist in the egosphere
			for(Info::Itr p = emotionItr; p; p++)
			{
				std::string egoEmotion = p->get<std::string>("name");
				if(egoEmotion.compare((*it).name) == 0)
				{
					//verify if the intensity has changed //we dont want a change in the intensity to provoke a new event in the episodic memory
//					double egoEmotionIntensity = p->get<double>("intensity");
//					if( round(egoEmotionIntensity) != round((*it).value) )
//					{
//						setStmChanged(true);
//					}
					//update the cache duration
					p->set( param );
					p->cacheDurationLimit(Info::Duration(getDuration()));
					isPresent = true;
				}
			}
			if(!isPresent)
			{
				//add the emotion to the egosphere
				Info::Ptr infoEmotion = egos_->newInfo("emotion", param);
				infoEmotion->cacheLimits(Info::Duration(getDuration()),10);
				setStmChanged(true);
			}
		}
	}
}
