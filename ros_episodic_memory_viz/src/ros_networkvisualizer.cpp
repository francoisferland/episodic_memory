/*
 * ros_networkvisualizer.cpp
 *
 *  Created on: 2012-10-25
 *      Author: frank
 */
#include "ros_networkvisualizer.h"

ROS_NetworkVisualizer::ROS_NetworkVisualizer( ros::NodeHandle* p_nodeHandle, QWidget *parent):NetworkVisualizer(parent)
{
	//getActionToolbar()->removeAction(randomEvent);

	nodeHandle = p_nodeHandle;
	inputDataPublisher = NULL;
	learningModePublisher = NULL;
	newWeight = 0;

	QObject::connect(this , SIGNAL( sendInputData(std::map<std::string,EM_CORE::InputROSmsgPtr>) ) ,
			this , SLOT( onPublishInputData(std::map<std::string,EM_CORE::InputROSmsgPtr> ) ) );

	QObject::connect(this, SIGNAL(clearActivationSignal()),
			this, SLOT(onPublishClearActivation()));

	QObject::connect(this, SIGNAL(toggleLearningMode(EM_CORE::LEARNING_MODE)),
			this, SLOT(onPublishLearningMode(EM_CORE::LEARNING_MODE)));

}

ROS_NetworkVisualizer::~ROS_NetworkVisualizer()
{
}

void ROS_NetworkVisualizer::updatedWeightSubscriber(const ros_episodic_memory::updatedWeight& message)
{
	onUpdateWeight(message.id, message.indexTopNode, message.indexBottomNode, message.topLayer, message.value);
	newWeight++;
	ROS_INFO("received %i weight (%i)",newWeight,message.id);
}

void ROS_NetworkVisualizer::newCategorySubscriber(const ros_episodic_memory::newCategory& message)
{
	onNewCategory(message.indexCategory, message.indexChannel, QString::fromStdString(message.description), message.vigilance, message.learningRate, message.layer);
}

void ROS_NetworkVisualizer::updateActivationSubscriber(const ros_episodic_memory::categoryActivation& message)
{
	onUpdateActivation(message.indexCategory, message.indexChannel, message.value, message.layer);
}

void ROS_NetworkVisualizer::newChannelSubscriber(const ros_episodic_memory::newChannel& message)
{
	onNewChannel(message.indexChannel, QString::fromStdString(message.description), message.relevance, message.layer);
}

void ROS_NetworkVisualizer::anticipatedEventSubscriber(const ros_episodic_memory::anticipatedEvent& message)
{
	ROS_INFO("Received anticipated event %i",message.index);
	onAnticipatedEvent(message.index, message.newVigilance, message.newLearningRate, message.anticipated);
}

void ROS_NetworkVisualizer::onMenuResetNetwork()
{
	NetworkVisualizer::onMenuResetNetwork();

	ROS_INFO("Weights cleared from the GUI");

	//use service for clearMemory
	ros::ServiceClient client = nodeHandle->serviceClient<std_srvs::Empty>("clear_em_memory");
	std_srvs::Empty empty;

	if(client.call(empty))
	{
		ROS_INFO("Request to clear episodic memory sent to episodic_memory_core");
	}

}

int ROS_NetworkVisualizer::onChangeVigilance()
{
	int indexCategory = NetworkVisualizer::onChangeVigilance();

	ros::ServiceClient client = nodeHandle->serviceClient<ros_episodic_memory::setVigilanceParameter>("set_vigilance_parameter");
	ros_episodic_memory::setVigilanceParameter vigilanceParameters;

	vigilanceParameters.request.vigilance = (float)widget->getVigilanceSpinbox(indexCategory)->value();
	vigilanceParameters.request.indexCategory = indexCategory;

	if(client.call(vigilanceParameters))
	{
		ROS_DEBUG("vigilance on category %i = %5.2f",indexCategory, vigilanceParameters.request.vigilance);
	}

	return indexCategory;

}

int ROS_NetworkVisualizer::onChangeLearningRate()
{
	int indexCategory = NetworkVisualizer::onChangeLearningRate();

	ros::ServiceClient client = nodeHandle->serviceClient<ros_episodic_memory::setLearningRateParameter>("set_learningRate_parameter");
	ros_episodic_memory::setLearningRateParameter learningRateParameters;

	learningRateParameters.request.learningRate = (float)widget->getLearningRateSpinbox(indexCategory)->value();
	learningRateParameters.request.indexCategory = indexCategory;

	if(client.call(learningRateParameters))
	{
		ROS_DEBUG("learningRate %i = %5.2f",indexCategory, learningRateParameters.request.learningRate);
	}

	return indexCategory;
}

int ROS_NetworkVisualizer::onChangeRelevance(double relevance)
{
	int indexCategory = NetworkVisualizer::onChangeRelevance(relevance);

	ros::ServiceClient client = nodeHandle->serviceClient<ros_episodic_memory::setRelevanceParameter>("set_relevance_parameter");
	ros_episodic_memory::setRelevanceParameter relevanceParameters;

	relevanceParameters.request.relevance = (float)widget->getRelevanceSpinbox(indexCategory)->value();
	relevanceParameters.request.indexCategory = indexCategory;

	if(client.call(relevanceParameters))
	{
		ROS_DEBUG("relevance channel %i = %5.2f",indexCategory, relevanceParameters.request.relevance);
	}

	return indexCategory;
}

void ROS_NetworkVisualizer::onPublishLearningMode(EM_CORE::LEARNING_MODE lm_mode)
{
	ros_episodic_memory::learningMode mode;
	if(lm_mode == EM_CORE::LEARNING)
		mode.learningMode = ros_episodic_memory::learningMode::LEARNING;
	else if(lm_mode == EM_CORE::RECOGNIZING)
		mode.learningMode = ros_episodic_memory::learningMode::RECOGNIZING;

	this->learningModePublisher->publish(mode);
}

void ROS_NetworkVisualizer::onPublishInputData(std::map<std::string,EM_CORE::InputROSmsgPtr> p_inputData)
{
	ros_episodic_memory::inputData inputData;

	//iterate through the map to fill the message
	for(std::map<std::string,EM_CORE::InputROSmsgPtr>::iterator it = p_inputData.begin() ; it != p_inputData.end() ; it++)
	{
		std::string categoryName = (*it).second->getDesc();
		float activationValue = (*it).second->getActivValue();
		std::string channelName = (*it).second->getChannelDesc();
		float channelRelevance = (*it).second->getMatchRelev();

		inputData.categoryName.push_back(categoryName);
		inputData.channelName.push_back(channelName);
		inputData.activationValue.push_back(activationValue);
		inputData.channelRelevance.push_back(channelRelevance);
	}

	this->inputDataPublisher->publish(inputData);
}

void ROS_NetworkVisualizer::onPublishClearActivation()
{
	ros::ServiceClient client = nodeHandle->serviceClient<std_srvs::Empty>("clear_all_activation");
	std_srvs::Empty obj;
	if(client.call(obj))
	{
		ROS_DEBUG("Request to clear all activation");
	}
}

void ROS_NetworkVisualizer::learningModeChangeSubscriber(const ros_episodic_memory::learningMode& msg)
{
	EM_CORE::LEARNING_MODE mode = EM_CORE::RECOGNIZING;

	if(msg.learningMode == ros_episodic_memory::learningMode::RECOGNIZING)
		mode = EM_CORE::RECOGNIZING;
	else if(msg.learningMode == ros_episodic_memory::learningMode::LEARNING)
		mode = EM_CORE::LEARNING;

	NetworkVisualizer::onChangeLearningMode(mode);

}

void ROS_NetworkVisualizer::callbackEmotion(const hbba_msgs::EmotionIntensitiesPtr& message)
{
	//get the max intensity from all the emotion generated
	float maxIntensity = 0;
	for(std::vector<emotions_msgs::Intensity>::iterator it =  message->emotion.begin() ; it != message->emotion.end() ; it++)
	{
		if((*it).value > maxIntensity)
			maxIntensity = (*it).value;
	}
	//update progress bar
	int emotionIntensity = maxIntensity * 100;
	this->onUpdateEmotionIntensity(emotionIntensity);
}
