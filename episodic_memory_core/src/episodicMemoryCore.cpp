#include "episodic_memory_core/episodicMemoryCore.h"

EpisodicMemoryCore::EpisodicMemoryCore(const char** databasePath)
{
	//instanciate the three layers for the network
	m_eventLayer = EventLayerPtr(new EventLayer());
	m_episodeLayer = EpisodeLayerPtr(new EpisodeLayer());
	m_inputLayer = InputLayerPtr(new InputLayer());

	m_inputLayer->setUpperLayer(m_eventLayer);
	//input layer doesnt have a lower layer
	m_eventLayer->setUpperLayer(m_episodeLayer);
	m_eventLayer->setLowerLayer(m_inputLayer);
	//episode layer does not have an upper layer
	m_episodeLayer->setLowerLayer(m_eventLayer);

	m_databasePath = databasePath;

	emotionIntensity_ = 0;

	db = NULL;
}

bool EpisodicMemoryCore::initialize()
{
	bool success = true;
	//create connection to the database
	int returnValue = sqlite3_open(*m_databasePath, &db);
	if( returnValue ){
		ROS_ERROR( "Can't open database in '%s': %s" , *m_databasePath, sqlite3_errmsg(db));
		sqlite3_close(db);
		success = false;
	}
	else
		success = true;

	if(success)
	{
		//create dao
		WeightDaoPtr weightDao = DaoFactory::getWeightDao(db);
		InputDaoPtr inputDao = DaoFactory::getInputDao(db);
		ChannelDaoPtr channelDao = DaoFactory::getChannelDao(db);
		PatternRecognizerDaoPtr patternRecognizerDao = DaoFactory::getPatternRecognizerDao(db);

		m_eventLayer->setDao(weightDao);
		m_eventLayer->setDao(channelDao);
		m_eventLayer->setDao(patternRecognizerDao);
		m_inputLayer->setDao(weightDao);
		m_inputLayer->setDao(inputDao);
		m_inputLayer->setDao(channelDao);
		m_episodeLayer->setDao(patternRecognizerDao);
		m_episodeLayer->setDao(channelDao);
		m_episodeLayer->setDao(weightDao);

		//load channel for each layer
		m_inputLayer->setListChannel(channelDao->findAll(m_inputLayer->getLayerId()));
		m_eventLayer->setListChannel(channelDao->findAll(m_eventLayer->getLayerId()));
		m_episodeLayer->setListChannel(channelDao->findAll(m_episodeLayer->getLayerId()));

		//std::map<int,ChannelARTptr> mapChannel = m_eventLayer->getMapChannel();
		//for(std::map<int,ChannelARTptr>::iterator it = mapChannel.begin() ; it!= mapChannel.end() ; it++)
		//{
		//	(*it).second->setDebugResonance(true);
		//}

		//load category for each channel (input, patternRecognizer)
		m_inputLayer->initializeCategories();
		m_eventLayer->initializeCategories();
		m_episodeLayer->initializeCategories();

		//load weight for each layer and link each weight to two category
		std::map<int,WeightObjPtr> listWeightF1F2 = weightDao->findAll(m_inputLayer->getLayerId(),m_inputLayer->getListCategory(),m_eventLayer->getListCategory());
		std::map<int,WeightObjPtr> listWeightF2F3 = weightDao->findAll(m_eventLayer->getLayerId(),m_eventLayer->getListCategory(),m_episodeLayer->getListCategory());
		m_inputLayer->setListWeight(listWeightF1F2);
		m_eventLayer->setListWeight(listWeightF2F3);

		//set the memory decay for the event layer
		m_eventLayer->setMemoryDecay(MemoryDecayPtr(new MemoryDecay()));

	}
	return success;
}

CategoryARTptr EpisodicMemoryCore::processInput()
{
	//set the emotion intensity in the two layers where a learning can occur
	m_eventLayer->setCurrentEmotionIntensity(emotionIntensity_);
	m_episodeLayer->setCurrentEmotionIntensity(emotionIntensity_);

	//resonance between input layer and event
	CategoryARTptr event = m_inputLayer->resonance();
	m_eventLayer->notifyPatternRecognizerParameter(event);

	//resonance between event layer and episode
	CategoryARTptr episode = m_eventLayer->resonance();
	m_episodeLayer->notifyPatternRecognizerParameter(episode);

	//reset vigilance to default values before modifying them
	m_eventLayer->resetVigilance();
	//get anticipated events if there is a resonance in the event layer and adjust vigilance threshold
	if(episode)
	{
		std::vector<CategoryARTptr> anticipatedEvents = m_episodeLayer->buildRecognizedEpisode(episode);
		adjustVigilance(anticipatedEvents);
		//notify the view for updated vigilance only
		m_eventLayer->notifyPatternRecognizerParameter();
		//notify the view for only anticipated event
		for(unsigned int i = 0 ; i < anticipatedEvents.size() ; i++)
		{
			PatternRecognizerObjPtr pattern = boost::dynamic_pointer_cast<PatternRecognizerObj>(anticipatedEvents[i]);
			m_eventLayer->notifyAnticipatedEvent(pattern);
		}
	}
	else
	{
		//notify the view that there is no active episode and no anticipated events
		m_episodeLayer->notifyActivation(true);
		//notify the view for updated vigilance and clear anticipated events
		m_eventLayer->notifyPatternRecognizerParameter();
	}

	if(m_episodeLayer->getLearningMode() == EM_CORE::LEARNING)
	{
		m_episodeLayer->setLearningMode(RECOGNIZING);
		m_episodeLayer->notifyLearningModeChanged();
		clearActivation();
	}

	return episode;
}

bool EpisodicMemoryCore::activateInputs(std::map<std::string,InputROSmsgPtr> inputROSmsgMap)
{
	bool success = true;
	//Adjust the network if there are new inputs and channel
	for(std::map<std::string,InputROSmsgPtr>::iterator it = inputROSmsgMap.begin(); it != inputROSmsgMap.end() ; it++)
	{
		std::string categoryName = (*it).second->getDesc();
		std::string channelName = (*it).second->getChannelDesc();
		//float activationValue = (*it).second->getActivValue();
		int channelRelevance = (*it).second->getMatchRelev();

		ChannelARTptr channel = this->getInputLayer()->getChannelByDescription(channelName);

		if(channelRelevance > 10)
			channelRelevance = 10;
		if(channelRelevance < 0)
			channelRelevance = 0;
		float relevance = (float)channelRelevance / 10.0;

		if(!channel)
			channel = addInputChannel(channelName, relevance);
		else
			channel->setRelevance(relevance);

		if(!channel)
			success = false;
		//find category by description
		InputObjPtr input = m_inputLayer->getCategoryByDescription(categoryName);

		if(success && !input )
			//create new input
			input = addInput(categoryName, channelName);
		else if(success)
			//update input
			input->setDescription(categoryName);

		if(!input)
			success = false;
	}

	//Activates input category and their complement
	if(success)
		success &= m_inputLayer->activateInputs(inputROSmsgMap);

	return success;
}

void EpisodicMemoryCore::publishAllWeight()
{
	m_inputLayer->notifyAllWeight();
	m_eventLayer->notifyAllWeight();
	m_episodeLayer->notifyAllWeight();
}

void EpisodicMemoryCore::publishAllCategories()
{
	m_inputLayer->notifyAllCategories();
	m_eventLayer->notifyAllCategories();
	m_episodeLayer->notifyAllCategories();
}

void EpisodicMemoryCore::publishAllChannels()
{
	m_inputLayer->notifyAllChannels();
	m_eventLayer->notifyAllChannels();
	m_episodeLayer->notifyAllChannels();
}

void EpisodicMemoryCore::publishAllActivation()
{
	m_inputLayer->notifyActivation(false);
	m_eventLayer->notifyActivation(false);
	m_episodeLayer->notifyActivation(false);
}

ChannelARTptr EpisodicMemoryCore::addInputChannel(std::string description, float relevance)
{
	return m_inputLayer->addChannel(description, relevance);
}

InputObjPtr EpisodicMemoryCore::addInput(std::string inputName, std::string channelDescription)
{
	return m_inputLayer->addNewInput(inputName,channelDescription);
}

void EpisodicMemoryCore::setLearningRateMode(LEARNING_MODE mode)
{
	ROS_INFO("core: receive learning rate mode %i\n", mode);
	m_episodeLayer->setLearningMode(mode);

	/*if(mode == EM_CORE::LEARNING)
	{
		//resonance between event layer and episode
		m_eventLayer->resonance();

		//learning is allowed only once. we must reactivated each time we want to learn a new episode pattern
		m_episodeLayer->setLearningMode(RECOGNIZING);
		m_inputLayer->clearActivation();
		m_eventLayer->clearActivation();
	}*/
	m_episodeLayer->notifyLearningModeChanged();
}

void EpisodicMemoryCore::adjustVigilance(std::vector<CategoryARTptr> anticipatedEvents)
{
	float maxDecrement = 0.2;
	int i = 0;
	int nbEvents = anticipatedEvents.size();
	for(std::vector<CategoryARTptr>::iterator it=anticipatedEvents.begin() ; it != anticipatedEvents.end(); it++)
	{
		PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>(*it);
		float newVigilance = patternRecognizer->getVigilance() - (maxDecrement - i * maxDecrement / nbEvents); i++;
		patternRecognizer->setVigilance(newVigilance);
	}
}

void EpisodicMemoryCore::setVigilance(int indexCategory, float value)
{
	//vigilance affects only event and episode layer
	CategoryARTptr tempCat;
	tempCat = m_eventLayer->getCategoryByIndex(indexCategory);
	if(!tempCat)
		tempCat = m_episodeLayer->getCategoryByIndex(indexCategory);

	if(tempCat)
	{
		PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>(tempCat);
		patternRecognizer->setVigilance(value);
		//update database
		PatternRecognizerDaoPtr patternRecognizerDao = DaoFactory::getPatternRecognizerDao(db);
		patternRecognizerDao->updateObj(patternRecognizer);
		patternRecognizerDao.reset();

	}
	else
	{
		ROS_WARN("Can't find the category to set the vigilance %i", indexCategory);
	}
}

void EpisodicMemoryCore::setLearningRate(int indexCategory, float value)
{
	//vigilance affects only event and episode layer
	CategoryARTptr tempCat;
	tempCat = m_eventLayer->getCategoryByIndex(indexCategory);
	if(!tempCat)
		tempCat = m_episodeLayer->getCategoryByIndex(indexCategory);

	if(tempCat)
	{
		PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>(tempCat);
		patternRecognizer->setLearningRate(value);
		//update database
		PatternRecognizerDaoPtr patternRecognizerDao = DaoFactory::getPatternRecognizerDao(db);
		patternRecognizerDao->updateObj(patternRecognizer);
		patternRecognizerDao.reset();
	}
	else
	{
		ROS_WARN("Can't find the category to set the learningRate");
	}
}

void EpisodicMemoryCore::setRelevance(int indexChannel, float value)
{
	ChannelARTptr channel;
	if(m_inputLayer->getMapChannel().count(indexChannel))
	{
		channel = m_inputLayer->getMapChannel().at(indexChannel);
		channel->setRelevance(value);
		m_inputLayer->updateChannelDatabase(channel);
	}
	else if(m_eventLayer->getMapChannel().count(indexChannel))
	{
		channel = m_eventLayer->getMapChannel().at(indexChannel);
		channel->setRelevance(value);
		m_eventLayer->updateChannelDatabase(channel);
	}
	else if(m_episodeLayer->getMapChannel().count(indexChannel))
	{
		channel = m_episodeLayer->getMapChannel().at(indexChannel);
		channel->setRelevance(value);
		m_episodeLayer->updateChannelDatabase(channel);
	}
	else
		ROS_INFO("Can't change relevance, channel %i doesnt exist in the core",indexChannel);
}

void EpisodicMemoryCore::printNetwork()
{

}

void EpisodicMemoryCore::onReceiveData(std::map<std::string,EM_CORE::InputROSmsgPtr> mapData)
{
	//process input through the network
	if(activateInputs(mapData))
		processInput();
}

void EpisodicMemoryCore::onEmotionChange(float emotionIntensity)
{
	//emotionIntensity must be normalized between 0 and 1
	if(emotionIntensity > 1)
	{
		emotionIntensity = 1;
	}
	else if(emotionIntensity < 0)
	{
		emotionIntensity = 0;
	}
ROS_INFO_THROTTLE(1,"emotionIntensity = %.2f\n",emotionIntensity);
	emotionIntensity_ = emotionIntensity;
}

std::vector<std::vector<CategoryARTptr> > EpisodicMemoryCore::recallLastActivatedEpisode()
{
	std::vector<std::vector<CategoryARTptr> > recalledEpisode;

	return recalledEpisode;
}

void EpisodicMemoryCore::clearEpisodicMemory()
{
	clearWeights();
	clearInputs();
	clearChannels();
	clearActivation();
}

void EpisodicMemoryCore::clearChannels()
{
	m_inputLayer->clearChannels();
	m_eventLayer->clearChannels();
	m_episodeLayer->clearChannels();
}

void EpisodicMemoryCore::clearActivation()
{
	m_inputLayer->clearActivation();
	m_eventLayer->clearActivation();
	m_episodeLayer->clearActivation();
}

void EpisodicMemoryCore::clearWeights()
{
	m_inputLayer->clearWeights();
	m_eventLayer->clearWeights();
	m_episodeLayer->clearWeights();
}

void EpisodicMemoryCore::clearInputs()
{
	m_inputLayer->clearInputs();
	m_eventLayer->clearPatternRecognizers();
	m_episodeLayer->clearPatternRecognizers();
}
