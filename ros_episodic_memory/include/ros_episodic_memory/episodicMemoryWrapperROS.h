#ifndef EPISODICMEMORYWRAPPERROS_H_
#define EPISODICMEMORYWRAPPERROS_H_

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <episodicMemoryCore.h>
#include <episodeLayer.h>
#include "ros_episodic_memory/updatedWeight.h"
#include "ros_episodic_memory/newCategory.h"
#include "ros_episodic_memory/newChannel.h"
#include "ros_episodic_memory/categoryActivation.h"
#include "ros_episodic_memory/setVigilanceParameter.h"
#include "ros_episodic_memory/setLearningRateParameter.h"
#include "ros_episodic_memory/setRelevanceParameter.h"
//#include "ros_episodic_memory/setLearningMode.h"
#include "ros_episodic_memory/inputData.h"
#include "ros_episodic_memory/learningMode.h"
#include "ros_episodic_memory/recalledEpisode.h"
#include "ros_episodic_memory/contextRepresentation.h"
#include "ros_episodic_memory/anticipatedEvent.h"
#include "perception_context_msgs/egosphereContent.h"
#include "hbba_msgs/EmotionIntensities.h"


class EpisodicMemoryWrapperROS : public QObject
{
	Q_OBJECT

  public:
	EpisodicMemoryWrapperROS(EM_CORE::EpisodicMemoryCorePtr);
	~EpisodicMemoryWrapperROS();
	
	void callbackShortTermMemory(const perception_context_msgs::egosphereContentPtr& message);

	void callbackEmotion(const hbba_msgs::EmotionIntensitiesPtr& message);

	void callbackLearningMode(const ros_episodic_memory::learningMode & message);

	void receiveInputData(const ros_episodic_memory::inputData& message);

	bool recallLastEpisode(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	//service
	bool clearEmMemory(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	bool getInitialValues(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	bool processRandomInput(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	bool vigilanceService(ros_episodic_memory::setVigilanceParameter::Request &req,
			ros_episodic_memory::setVigilanceParameter::Response &response);
	bool learningRateService(ros_episodic_memory::setLearningRateParameter::Request &req,
			ros_episodic_memory::setLearningRateParameter::Response &response);
	bool relevanceService(ros_episodic_memory::setRelevanceParameter::Request &req,
				ros_episodic_memory::setRelevanceParameter::Response &response);
//	bool learningModeService(ros_episodic_memory::setLearningMode::Request &req,
//					ros_episodic_memory::setLearningMode::Response &response);

	bool clearActivationService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

	//publishers
	void setUpdatedWeightPublisher(ros::Publisher* pubPtr){ this->m_updatedWeightPublisher = pubPtr;}
	void setNewCategoryPublisher(ros::Publisher* pubPtr){ this->m_newCategoryPublisher = pubPtr;}
	void setCategoryActivationPublisher(ros::Publisher* pubPtr){ this->m_updateActivationPublisher = pubPtr;}
	void setNewChannelPublisher(ros::Publisher* pubPtr){ this->m_newChannelPublisher = pubPtr;}
	void setAnticipatedEventPublisher(ros::Publisher* pubPtr){ this->m_anticipatedEventPublisher = pubPtr;}
	void setRecallEpisodePublisher(ros::Publisher* pubPtr){ this->m_recallEventPublisher = pubPtr;}
	void setLearningModePublisher(ros::Publisher* pubPtr){ this->m_learningModePublisher = pubPtr;}

  public slots:

	void pubNewCategory(int index, int channelId, QString description, float vigilance, float learningRate, int layerID);
	void pubUpdateActivation(int index, int idChannel, float value, int idLayer);
	void pubUpdateWeight(int id, int idTopNode, int idBottomNode, int topLayerId, float weightValue);
	void pubNewChannel(int channelId, QString description, float relevance, int layer);
	void pubAnticipatedEvent(int index, float newVigilance, float newLearningRate, bool anticipated);

	void pubRecognizedEpisode( std::vector<CategoryARTptr> );
	void pubLearningModeChanged(EM_CORE::LEARNING_MODE);

  private:
	EM_CORE::EpisodicMemoryCorePtr m_episodicMemoryPtr;

	ros::Publisher* m_updatedWeightPublisher;
	ros::Publisher* m_newCategoryPublisher;
	ros::Publisher* m_updateActivationPublisher;
	ros::Publisher* m_newChannelPublisher;
	ros::Publisher* m_anticipatedEventPublisher; //for gui only
	ros::Publisher* m_recallEventPublisher;		 //for other nodes
	ros::Publisher* m_learningModePublisher;
	int newWeight;
};

#endif
