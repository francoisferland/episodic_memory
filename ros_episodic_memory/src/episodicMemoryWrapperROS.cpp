#include <episodicMemoryWrapperROS.h>

EpisodicMemoryWrapperROS::EpisodicMemoryWrapperROS(EpisodicMemoryCorePtr emPtr): m_episodicMemoryPtr(emPtr)
{
	m_updatedWeightPublisher = NULL;
	m_newCategoryPublisher = NULL;
	m_newChannelPublisher = NULL;
	m_updateActivationPublisher = NULL;
	m_anticipatedEventPublisher = NULL;
	m_recallEventPublisher = NULL;
	m_learningModePublisher = NULL;
	newWeight = 0;

}

EpisodicMemoryWrapperROS::~EpisodicMemoryWrapperROS()
{
}

void EpisodicMemoryWrapperROS::callbackShortTermMemory(const perception_context_msgs::egosphereContentPtr& message)
{
	ROS_DEBUG("Inside callbackShortTermMemory");

	//convert messages
	ros_episodic_memory::inputData inputData;
	inputData.categoryName = message->item;
	inputData.channelName = message->channel;
	inputData.activationValue = message->value;
	inputData.channelRelevance = message->matchRelevance;
	//process in the episodic memory
	receiveInputData(inputData);

}

void EpisodicMemoryWrapperROS::callbackEmotion(const hbba_msgs::EmotionIntensitiesPtr& message)
{
	//get the max intensity from all the emotion generated
	float maxIntensity = 0;
	for(std::vector<emotions_msgs::Intensity>::iterator it =  message->emotion.begin() ; it != message->emotion.end() ; it++)
	{
		if((*it).value > maxIntensity)
			maxIntensity = (*it).value;
	}

	m_episodicMemoryPtr->onEmotionChange(maxIntensity);
}

void EpisodicMemoryWrapperROS::callbackLearningMode(const ros_episodic_memory::learningMode & message)
{
	EM_CORE::LEARNING_MODE mode = EM_CORE::RECOGNIZING; // default

	if(message.learningMode == ros_episodic_memory::learningMode::RECOGNIZING)
	{
		mode = EM_CORE::RECOGNIZING;
	}
	else if(message.learningMode == ros_episodic_memory::learningMode::LEARNING)
	{
		mode = EM_CORE::LEARNING;
	}

	//ROS_INFO("receive force change learning mode %i",mode);
	this->m_episodicMemoryPtr->setLearningRateMode(mode);

}

bool EpisodicMemoryWrapperROS::clearEmMemory(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

	this->m_episodicMemoryPtr->clearEpisodicMemory();

	return true;
}

bool EpisodicMemoryWrapperROS::processRandomInput(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

	return true;
}

void EpisodicMemoryWrapperROS::receiveInputData(const ros_episodic_memory::inputData& message)
{

	//rebuild the map to send to the core
	std::map<std::string,EM_CORE::InputROSmsgPtr> mapInputData;
	if(message.activationValue.size() == message.categoryName.size() &&
			message.activationValue.size() == message.channelName.size())
	{
		for(unsigned int i = 0 ; i < message.categoryName.size() ; i++)
		{
			mapInputData[message.categoryName[i]] = EM_CORE::InputROSmsgPtr(new EM_CORE::InputROSmsg(message.categoryName[i],message.channelName[i],message.activationValue[i],message.channelRelevance[i]));
		}
	}
	else
	{
		ROS_WARN("can't send the input data to the core because the lists are not the same size");
	}
	this->m_episodicMemoryPtr->onReceiveData(mapInputData);
}

bool EpisodicMemoryWrapperROS::recallLastEpisode(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	/*ROS_INFO("Try to recall last activated memory");
	//build the episode with all the events composed of inputs
	std::vector<EM_CORE::RecalledEvent> recalledEpisode = this->m_episodicMemoryPtr->recallLastActivatedEpisode();
	//iterate through the events and publish one event at a time
	for(unsigned int i = 0 ; i < recalledEpisode.size() ; i++)
	{
		ros_episodic_memory::recalledEvent recalledEvent;
		recalledEvent.indexEvent = recalledEpisode[i].realIndex;
		recalledEvent.listInputDescription = recalledEpisode[i].listInput;
		m_recallEventPublisher->publish(recalledEvent);
	}*/
	return true;
}

bool EpisodicMemoryWrapperROS::getInitialValues(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	//ask the episodic memory to publish all its content
	this->m_episodicMemoryPtr->publishAllChannels();
	this->m_episodicMemoryPtr->publishAllCategories();
	this->m_episodicMemoryPtr->publishAllWeight();
	this->m_episodicMemoryPtr->publishAllActivation();

	return true;
}

bool EpisodicMemoryWrapperROS::vigilanceService(ros_episodic_memory::setVigilanceParameter::Request &req,
		ros_episodic_memory::setVigilanceParameter::Response &response)
{
	ROS_DEBUG("vigilance = %i = %5.2f", req.indexCategory, req.vigilance);

	this->m_episodicMemoryPtr->setVigilance(req.indexCategory, req.vigilance);

	return true;
}

bool EpisodicMemoryWrapperROS::learningRateService(ros_episodic_memory::setLearningRateParameter::Request &req,
		ros_episodic_memory::setLearningRateParameter::Response &response)
{
	ROS_DEBUG("learningRate = %i = %5.2f", req.indexCategory, req.learningRate);

	this->m_episodicMemoryPtr->setLearningRate(req.indexCategory, req.learningRate);

	return true;
}

bool EpisodicMemoryWrapperROS::relevanceService(ros_episodic_memory::setRelevanceParameter::Request &req,
		ros_episodic_memory::setRelevanceParameter::Response &response)
{
	ROS_DEBUG("relevance = %i = %5.2f", req.indexCategory, req.relevance);

	this->m_episodicMemoryPtr->setRelevance(req.indexCategory, req.relevance);

	return true;
}

void EpisodicMemoryWrapperROS::pubUpdateWeight(int id, int idTopNode, int idBottomNode, int topLayerId, float weightValue)
{
	//publish on the ROS network
	ros_episodic_memory::updatedWeight updatedWeight;

	updatedWeight.id = id;
	updatedWeight.indexTopNode = idTopNode;
	updatedWeight.indexBottomNode = idBottomNode;
	updatedWeight.topLayer = topLayerId;
	updatedWeight.value = weightValue;

	this->m_updatedWeightPublisher->publish(updatedWeight);
//	newWeight++;
//	ROS_INFO("%i weight sent so far (%i)",newWeight,id);
}

void EpisodicMemoryWrapperROS::pubNewCategory(int index, int channelId, QString description, float vigilance, float learningRate, int layerId)
{
	ros_episodic_memory::newCategory newCategory;

	newCategory.indexCategory = index;
	newCategory.indexChannel = channelId;
	newCategory.description = description.toStdString();
	newCategory.layer = layerId;
	newCategory.vigilance = vigilance;
	newCategory.learningRate = learningRate;

	this->m_newCategoryPublisher->publish(newCategory);
}

void EpisodicMemoryWrapperROS::pubNewChannel(int channelId, QString description, float relevance, int layer)
{
	ros_episodic_memory::newChannel newChannel;

	newChannel.indexChannel = channelId;
	newChannel.description = description.toStdString();
	newChannel.relevance = relevance;
	newChannel.layer = layer;

	this->m_newChannelPublisher->publish(newChannel);
}

void EpisodicMemoryWrapperROS::pubUpdateActivation(int index, int idChannel, float value, int idLayer)
{
	ros_episodic_memory::categoryActivation activation;

	activation.indexCategory = index;
	activation.indexChannel = idChannel;
	activation.value = value;
	activation.layer = idLayer;

	this->m_updateActivationPublisher->publish(activation);
}

void EpisodicMemoryWrapperROS::pubAnticipatedEvent(int index, float newVigilance, float newLearningRate, bool anticipated)
{
	ros_episodic_memory::anticipatedEvent anticipatedEvent;

	anticipatedEvent.index = index;
	anticipatedEvent.newVigilance = newVigilance;
	anticipatedEvent.newLearningRate = newLearningRate;
	anticipatedEvent.anticipated = anticipated;

	this->m_anticipatedEventPublisher->publish(anticipatedEvent);
}

void EpisodicMemoryWrapperROS::pubRecognizedEpisode(std::vector<CategoryARTptr> recalledEpisode)
{
	ros_episodic_memory::recalledEpisode event;

	for( std::vector<CategoryARTptr>::iterator it = recalledEpisode.begin() ;
			it != recalledEpisode.end() ; it++ )
	{
		//find the weight value associated to the winner node
		float eventActivatedValue = 0;
		std::map<int,WeightObjPtr> mapUpWeight = (*it)->getMapUpWeight();
		for(std::map<int,WeightObjPtr>::iterator itWeight = mapUpWeight.begin() ; itWeight != mapUpWeight.end() ; itWeight++)
		{
			if((*itWeight).second->getUpCategoryPtr()->getActivationValue() > 0 )
			{
				eventActivatedValue = (*itWeight).second->getValue();
				break;
			}
		}

		event.realIndex.push_back((*it)->getIndexCategory());
		ros_episodic_memory::contextRepresentation context;
		std::map<int,WeightObjPtr> mapDownWeight = (*it)->getMapDownWeight();
		for(std::map<int,WeightObjPtr>::iterator itWeight = mapDownWeight.begin() ; itWeight != mapDownWeight.end() ; itWeight++)
		{
			//get the connected input category
			CategoryARTptr inputCat = (*itWeight).second->getDownCategoryPtr();
			InputObjPtr input = boost::dynamic_pointer_cast<InputObj>(inputCat); //input have to be a non-complement value

			if((*itWeight).second->getValue() > 0 && input->getComplementInput())
			{
				InputObjPtr input = boost::dynamic_pointer_cast<InputObj>((*itWeight).second->getDownCategoryPtr());
				context.listInputDescription.push_back(input->getDescription());
				context.listAssociatedChannel.push_back(input->getChannel()->getDescription());
				context.listActivationValue.push_back((*itWeight).second->getValue());
				context.eventActivationValue = eventActivatedValue;
			}
		}
		event.listInput.push_back(context);
	}

	if(event.realIndex.size() > 0)
		m_recallEventPublisher->publish(event);

}

void EpisodicMemoryWrapperROS::pubLearningModeChanged(EM_CORE::LEARNING_MODE mode)
{
	ros_episodic_memory::learningMode lm_msg;
	if(mode == EM_CORE::LEARNING)
		lm_msg.learningMode = ros_episodic_memory::learningMode::LEARNING;
	else if(mode == EM_CORE::RECOGNIZING)
		lm_msg.learningMode = ros_episodic_memory::learningMode::RECOGNIZING;

	this->m_learningModePublisher->publish(lm_msg);
}

bool EpisodicMemoryWrapperROS::clearActivationService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	this->m_episodicMemoryPtr->clearActivation();
	return true;
}
