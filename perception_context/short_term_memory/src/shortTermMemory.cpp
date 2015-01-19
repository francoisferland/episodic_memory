/*
 * shortTermMemory.cpp
 *
 *  Created on: 2013-03-05
 *      Author: frank
 */

#include <ros/ros.h>
#include "egosphere_ros/egosphere.hpp"
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include "perception_context_msgs/egosphereContent.h"
#include "perception_context_msgs/stmContent.h"
#include "stmPeople.h"
#include "stmObjects.h"
#include "stmLocation.h"
#include "stmDesires.h"
#include "stmEmotions.h"

using namespace egosphere_ros;

typedef typename boost::shared_ptr<Egosphere> EgosPtr;

ros::Publisher pubEgosphereContent;

struct ConfigStm
{
	struct configChannel
	{
		std::string name;
		int relevance;
	};
	std::vector<configChannel> channels; //list of channel defined by <name,relevance>
	int duration;
};

class ShortTermMemory
{
public :

	ShortTermMemory(EgosPtr egosphere, ros::NodeHandle n)
	{
		egos_ = egosphere;

		//load config_stm
		parseYAMLconfigFile();

		stmLocation_ = new StmLocation(egos_,stmConfigLocation_.duration);
		stmObjects_ = new StmObjects(egos_,n, stmConfigObject_.duration);
		stmPeople_  = new StmPeople(egos_, stmConfigPeople_.duration);
		stmDesires_ =  new StmDesires(egos_,&n, stmConfigDesire_.duration);
		stmEmotion_ = new StmEmotion(egos_,stmConfigEmotion_.duration);

		subLocation = n.subscribe<perception_context_msgs::locationDetectionConstPtr>("location_label",1,&StmLocation::updateData,stmLocation_);
		subObject = n.subscribe<std_msgs::Float32MultiArrayConstPtr>("objects",1,&StmObjects::updateData,stmObjects_);
		subPerson = n.subscribe<std_msgs::StringConstPtr>("person_identified",1,&StmPeople::updateData,stmPeople_);
		subEmotion = n.subscribe<hbba_msgs::EmotionIntensitiesConstPtr>("emotions",10,&StmEmotion::updateData,stmEmotion_);

		hasChanged = false;
	}

	void parseYAMLconfigFile(XmlRpc::XmlRpcValue param, std::vector<ConfigStm::configChannel> & channel, int & duration)
	{
		for(int nbStmParam = 0 ; nbStmParam < param.size() ; nbStmParam++)
		{
			//ROS_WARN("%i/%i",nbStmParam,param.size());
			for (std::map<std::string, XmlRpc::XmlRpcValue>::iterator it = param[nbStmParam].begin(); it != param[nbStmParam].end(); it++)
			{
				if((*it).first.compare("channel") == 0 && (*it).second.getType() == XmlRpc::XmlRpcValue::TypeArray)
				{
					XmlRpc::XmlRpcValue channelParam = (*it).second;
					//ROS_INFO("nb channel %i",channelParam.size());

					for(int i = 0 ; i < channelParam.size() ; i++)
					{
						for(std::map<std::string, XmlRpc::XmlRpcValue>::iterator it2 = channelParam[i].begin(); it2 != channelParam[i].end(); it2++)
						{
							ConfigStm::configChannel tempChannel;
							tempChannel.name = (*it2).first;
							tempChannel.relevance = static_cast<int>((*it2).second);
							channel.push_back( tempChannel );
							ROS_INFO("channel: name:%s, relevance:%i",(*it2).first.c_str(),static_cast<int>((*it2).second));
						}
					}

				}
				else if((*it).first.compare("duration") == 0 && (*it).second.getType() == XmlRpc::XmlRpcValue::TypeInt)
				{

					duration = static_cast<int>((*it).second);
					ROS_INFO("duration : %i\n",duration);
				}
			}
		}
	}

	void parseYAMLconfigFile()
	{
		ros::NodeHandle np("~");
		XmlRpc::XmlRpcValue stmPeopleConfig;
		XmlRpc::XmlRpcValue stmObjectConfig;
		XmlRpc::XmlRpcValue stmLocationConfig;
		XmlRpc::XmlRpcValue stmDesireConfig;
		XmlRpc::XmlRpcValue stmEmotionConfig;

		if(np.getParam("stmPeople", stmPeopleConfig))
			parseYAMLconfigFile(stmPeopleConfig, stmConfigPeople_.channels, stmConfigPeople_.duration);
		if(np.getParam("stmObjects", stmObjectConfig))
			parseYAMLconfigFile(stmObjectConfig, stmConfigObject_.channels, stmConfigObject_.duration);
		if(np.getParam("stmDesires", stmDesireConfig))
			parseYAMLconfigFile(stmDesireConfig, stmConfigDesire_.channels, stmConfigDesire_.duration);
		if(np.getParam("stmLocation", stmLocationConfig))
			parseYAMLconfigFile(stmLocationConfig, stmConfigLocation_.channels, stmConfigLocation_.duration);
		if(np.getParam("stmEmotions", stmEmotionConfig))
			parseYAMLconfigFile(stmEmotionConfig, stmConfigEmotion_.channels, stmConfigEmotion_.duration);

	}

	void verifyExpiration()
	{
	try
	{
		for(Info::Itr p = egos_->infoByTag("location"); p; p++)
		{
			ros::Time timestamp = p->getTimeStamp("room");
			int delay = ros::Time::now().sec - timestamp.sec;
			if(delay >= stmLocation_->getDuration() )
			{
				p->expire();
				ROS_DEBUG("location expire %is ",delay);
				hasChanged = true;
			}
		}
		for(Info::Itr p = egos_->infoByTag("person"); p; p++)
		{
			//verify if the location changed. if so, people in the short term memory will disappear more rapidly
			ros::Time timestamp = p->getTimeStamp("name");
			int delay = ros::Time::now().sec - timestamp.sec;
			if( delay >= stmPeople_->getDuration() || (stmLocation_->isStmChanged() && delay > 3) )
			{
				p->expire();
				ROS_DEBUG("person expire %is",delay);
				hasChanged = true;
			}
		}
		for(Info::Itr p = egos_->infoByTag("object"); p; p++)
		{
			ros::Time timestamp = p->getTimeStamp("objectId");
			int delay = ros::Time::now().sec - timestamp.sec;
			//if the object is associate with a grasp desire, dont expire the object
			try{
			Info::Itr desire = egos_->infoByAssociation(p, "exploited_desires");
			if(desire) {
				std::string desireGrasp = desire->get<std::string>("id");
				if(desireGrasp.compare("grasp") == 0)
				{
					break;
				}
			}
			}catch(egosphere::ParameterNotDefined &e){
				ROS_WARN("parameter not defined in verify expire object");
			}

			if( delay >= stmObjects_->getDuration() || (stmLocation_->isStmChanged() && delay > 3) )
			{
				p->expire();
				ROS_DEBUG("object expire %is",delay);
				hasChanged = true;
			}
		}
		for(Info::Itr p = egos_->infoByTag("objectRedundancy"); p; p++)
		{
			ros::Time timestamp = p->getTimeStamp("objectId");
			int delay = ros::Time::now().sec - timestamp.sec;
			if( delay >= stmObjects_->getDuration() || (stmLocation_->isStmChanged() && delay > 3))
			{
				p->expire();
			}
		}
		/*for(Info::Itr p = egos_->infoByTag("desires"); p; p++)
		{
			ros::Time timestamp = p->getTimeStamp("id");
			int delay = ros::Time::now().sec - timestamp.sec;
			if( delay >= stmDesires_->getDuration() )
			{
				p->expire();
			}
		}*/
		
	}
	catch(egosphere::ParameterNotDefined & e)
		{
			ROS_WARN("parameter not defined verify expiration");
		}

	}

	perception_context_msgs::egosphereContentPtr buildContentForPublish()
	{
		perception_context_msgs::egosphereContentPtr content = perception_context_msgs::egosphereContentPtr(new perception_context_msgs::egosphereContent);
		for(Info::Itr p = egos_->infoByTag("location"); p; p++)
		{
			std::string room;
			try
			{
				room = p->get<std::string>("room");
				content->item.push_back( room );
				content->value.push_back(1.0);
				content->channel.push_back(stmConfigLocation_.channels[0].name);
				content->matchRelevance.push_back(stmConfigLocation_.channels[0].relevance);

				std::string section = p->get<std::string>("section");
				if(!section.empty() && stmConfigLocation_.channels.size() > 1)
				{
					content->item.push_back( section );
					content->value.push_back(1.0);
					content->channel.push_back(stmConfigLocation_.channels[1].name);
					content->matchRelevance.push_back(stmConfigLocation_.channels[1].relevance);
				}

			}
			catch(egosphere::ParameterNotDefined & e)
			{
				ROS_INFO("no section in this room %s",room.c_str());
			}
		}

		for(Info::Itr p = egos_->infoByTag("object"); p; p++)
		{
			//convert objectId to string
			try{
				std::string s = boost::lexical_cast<std::string>(p->get<float>("objectId"));
				content->item.push_back( s );
				content->value.push_back(1.0);
				content->channel.push_back(stmConfigObject_.channels[0].name);
				content->matchRelevance.push_back(stmConfigObject_.channels[0].relevance);
			}
			catch(egosphere::ParameterNotDefined &e)
			{
			}
		}

		for(Info::Itr p = egos_->infoByTag("person"); p; p++)
		{
			//ROS_INFO("%s ",(*p).get<std::string>("name").c_str());
			try
			{
				content->item.push_back( p->get<std::string>("name") );
				content->value.push_back(1.0);
				content->channel.push_back(stmConfigPeople_.channels[0].name);
				content->matchRelevance.push_back(stmConfigPeople_.channels[0].relevance);
			}
			catch(egosphere::ParameterNotDefined &e)
			{
			}
		}

		for(Info::Itr p = egos_->infoByTag("exploited_desires"); p; p++)
		{
			//ROS_INFO("%s ",(*p).get<std::string>("id").c_str());
			try
			{
				content->item.push_back( p->get<std::string>("id") );
				content->value.push_back(0.85);
				content->channel.push_back(stmConfigDesire_.channels[0].name);
				content->matchRelevance.push_back(stmConfigDesire_.channels[0].relevance);
			}
			catch(egosphere::ParameterNotDefined &e)
			{
			}

		}

		for(Info::Itr p = egos_->infoByTag("emotion") ; p; p++)
		{
			try
			{
				content->item.push_back( p->get<std::string>("name") );
				content->value.push_back(p->get<double>("intensity"));
				content->channel.push_back(stmConfigEmotion_.channels[0].name);
				content->matchRelevance.push_back(stmConfigEmotion_.channels[0].relevance);
			}
			catch(egosphere::ParameterNotDefined &e)
			{
			}
		}
		//display content to send to episodic memory
		ROS_INFO("---- STM to EM -----");
		for(std::vector<std::string>::iterator it = content->item.begin() ; it != content->item.end() ; it++)
		{
			ROS_INFO("%s",(*it).c_str());
		}
		
		return content;
	}

	void publishContent()
	{
		hasChanged |= stmLocation_->isStmChanged();
		hasChanged |= stmPeople_->isStmChanged();
		hasChanged |= stmObjects_->isStmChanged();
		hasChanged |= stmDesires_->isStmChanged();
		hasChanged |= stmEmotion_->isStmChanged();

		if(hasChanged)
		{
			perception_context_msgs::egosphereContentPtr content = buildContentForPublish();
			if(content->item.size() > 0)
				pubEgosphereContent.publish(content);
			hasChanged = false;
			stmLocation_->setStmChanged(hasChanged);
			stmPeople_->setStmChanged(hasChanged);
			stmObjects_->setStmChanged(hasChanged);
			stmDesires_->setStmChanged(hasChanged);
			stmEmotion_->setStmChanged(hasChanged);
		}
	}

	bool publishContent(perception_context_msgs::stmContent::Request& req, perception_context_msgs::stmContent::Response& res)
	{
		res.content = *buildContentForPublish();
		return true;
	}

private :

	EgosPtr egos_;
	StmLocation * stmLocation_;
	StmPeople * stmPeople_;
	StmObjects * stmObjects_;
	StmDesires * stmDesires_;
	StmEmotion* stmEmotion_;

	ros::Subscriber subLocation;
	ros::Subscriber subObject;
	ros::Subscriber subPerson;
	//ros::Subscriber subDesire; //defined inside the stmDesires object
	ros::Subscriber subEmotion;

	//confidFile
	ConfigStm stmConfigPeople_;
	ConfigStm stmConfigObject_;
	ConfigStm stmConfigDesire_;
	ConfigStm stmConfigLocation_;
	ConfigStm stmConfigEmotion_;

	bool hasChanged;
};

int main(int argc, char *argv[])
{

	ros::init(argc, argv, "ShortTermMemory");

	ros::NodeHandle n;
	EgosPtr egosPtr = EgosPtr(new Egosphere (n, "/egosphere"));

	ShortTermMemory stm(egosPtr,n);

	ros::ServiceServer whatsInStm = n.advertiseService("stm_content",&ShortTermMemory::publishContent,&stm);

	pubEgosphereContent = n.advertise<perception_context_msgs::egosphereContent>("stm_content",true);

	ros::Rate loop_rate(5);

	while(ros::ok())
	{
		ros::spinOnce();

		//verfiy all items for expiring time
		stm.verifyExpiration();
		//publish current egosphere info
		stm.publishContent();

		loop_rate.sleep();
	}

	return 0;
}
