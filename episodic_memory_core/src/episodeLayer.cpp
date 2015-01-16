#include "episodeLayer.h"

EpisodeLayer::EpisodeLayer(): LayerART()
{
	m_layerId = EPISODE_LAYER;
}

EpisodeLayer::~EpisodeLayer()
{
}

CategoryARTptr EpisodeLayer::addCategory()
{
	CategoryARTptr category;
	ChannelARTptr channel = getChannelByDescription("episodeLayerChannel");
	if(channel)
	{
		category = CategoryARTptr(new PatternRecognizerObj());
		PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>(category);
		patternRecognizer->setChannel(channel);
		patternRecognizer->setIndexCategory(generateIndexCategory());
		//set learningRate according to current emotionIntensity
		patternRecognizer->computeLearningRate(currentEmotionIntensity_,false);

		//updateDatabase
		if(updatePatternRecognizerDatabase(patternRecognizer))
		{
			channel->addInput(patternRecognizer);
		}
		else
		{
			category.reset();
			return category;
		}

		//set the max activation value
		category->setActivationValue(1);

		notifyNewCategory(patternRecognizer);
	}

	return category;
}

void EpisodeLayer::computeVigilance(CategoryARTptr & category)
{
	PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>(category);
	if(patternRecognizer)
	{
		patternRecognizer->computeVigilance(currentEmotionIntensity_);
		updatePatternRecognizerDatabase(patternRecognizer);
	}
}

bool EpisodeLayer::learnPattern(CategoryARTptr & category)
{
	LayerART::learnPattern(category);

	//alter vigilance parameter according to current emotion intensity
	computeVigilance(category);

	return true;
}

bool EpisodeLayer::updatePatternRecognizerDatabase(PatternRecognizerObjPtr patternRecognizerObj)
{
	bool success = true;
	
	if(!patternRecognizerObj)
		return false;
	//update if the uid is known
	if(patternRecognizerObj->getUid() != -1)
	{
		success &= m_patternRecognizerDao->updateObj(patternRecognizerObj);
	}
	else
	{
		//create a database channel object
		success &= m_patternRecognizerDao->insertObj(patternRecognizerObj);
	}

	return success;
}

template<class T>
struct index_comparator {
	index_comparator(const T arr) : arr(arr) {}
	bool operator()(const size_t a, const size_t b) const
	{ return arr[a] > arr[b]; }

	const T arr;
};

std::vector<CategoryARTptr> sortEvents(std::vector<CategoryARTptr> vectActivatedEvent)
		{
	//index vector is defined by the first element of the pair
	std::vector<CategoryARTptr> eventsInOrder;
	std::vector<int> relativeIndexVect;
	std::vector<double> valueVect;
	for(unsigned int i = 0 ; i < vectActivatedEvent.size() ; i++)
	{
		relativeIndexVect.push_back(i);
		valueVect.push_back(vectActivatedEvent[i]->getActivationValue());
	}
	std::stable_sort(relativeIndexVect.begin() , relativeIndexVect.end() , index_comparator<std::vector<double> &>(valueVect));

	for(std::vector<int>::iterator it = relativeIndexVect.begin() ; it != relativeIndexVect.end() ; it++)
	{
		eventsInOrder.push_back(vectActivatedEvent[*it]);
	}

	return eventsInOrder;
		}

std::vector<CategoryARTptr> EpisodeLayer::buildRecognizedEpisode(CategoryARTptr winnerNode)
{
	std::vector<CategoryARTptr> anticipatedEvents;
	std::vector<CategoryARTptr> activatedEvents;

	//get the weight of the last activated event connected to the wining episode
	//CategoryARTptr lastActivatedEvent = m_lowerLayer->getLastActivatedCategory();
	CategoryARTptr lattestActivatedCategory = getLatestActivatedCategory(winnerNode);
	WeightObjPtr weight = lattestActivatedCategory->findWeightByConnection(winnerNode);
	float weightValue = 0 ;
	if(weight)
	{
		weightValue = weight->getValue();
	}

	std::map<int,WeightObjPtr> mapF2F3Weight = winnerNode->getMapDownWeight();
	for(std::map<int,WeightObjPtr>::iterator it = mapF2F3Weight.begin() ; it != mapF2F3Weight.end() ; it++)
	{
		if((*it).second->getValue() > 0 && (*it).second->getValue() >= weightValue)
		{
			CategoryARTptr event = (*it).second->getDownCategoryPtr();
			activatedEvents.push_back(event);
		}
	}

	//sort activatedCategory
	std::vector<CategoryARTptr> sortedActivatedEvents = sortEvents(activatedEvents);
	for(std::vector<CategoryARTptr>::iterator it = sortedActivatedEvents.begin() ; it != sortedActivatedEvents.end() ; it++)
	{
		if((*it)->getActivationValue() == 0)
		{
			anticipatedEvents.push_back(*it);
		}
	}

	displayBuiltEpisode(anticipatedEvents);

	emit episodeRecognised(anticipatedEvents);

	return anticipatedEvents;
}

void EpisodeLayer::displayBuiltEpisode(std::vector<CategoryARTptr> episode)
{
	if(episode.size() > 0)
		ROS_INFO("\n ---EPISODE --- \n");
	for(std::vector<CategoryARTptr>::iterator it = episode.begin() ; it!= episode.end() ; it++)
	{
		EventLayerPtr eventLayer = boost::dynamic_pointer_cast<EventLayer>(m_lowerLayer);
		std::vector<CategoryARTptr> inputPattern = eventLayer->getInputPattern(*it);
		ROS_INFO("---Event---\n");
		for(std::vector<CategoryARTptr>::iterator it2 = inputPattern.begin() ; it2 != inputPattern.end() ; it2++)
		{
			InputObjPtr input = boost::dynamic_pointer_cast<InputObj>(*it2);
			ROS_INFO("   %s",input->getDescription().c_str());
			ROS_INFO("\n");
		}
		ROS_INFO("\n");
	}
}

int EpisodeLayer::generateIndexCategory()
{
	int index = -1;

	index = this->getListCategory().size();
	index += m_lowerLayer->getListCategory().size();
	index += m_lowerLayer->getLowerLayer()->getListCategory().size();

	return index;
}

std::vector<CategoryARTptr> EpisodeLayer::recallLastActivatedEpisode()
{
	std::vector<CategoryARTptr> lastActivatedEpisode;
	/*if(mapChannel.size() == 0)
	{
		UINFO("no episode to recall");
		return lastActivatedEpisode;
	}
	//find the index of the last activated episode
	int index = 0;
	time_duration lastActivation = not_a_date_time;
	if(this->mapChannel[0]->getMapCategories()->count(0))
	{	//init to a real value
		lastActivation = this->mapChannel[0]->getMapCategories()->at(0)->getElapseTime();
	}
	for(std::map<int,CategoryARTptr>::iterator it = this->mapChannel[0]->getMapCategories()->begin(); it != this->mapChannel[0]->getMapCategories()->end() ; it++)
	{
		UINFO("event %i , elapse time %is",it->second->getIdNumber(),it->second->getElapseTime().seconds());
		if((*it).second->getElapseTime() < lastActivation)
		{
			index = (*it).second->getIdNumber();
			(*it).second->getElapseTime() = lastActivation;
		}
	}
	lastActivatedEpisode = recallEpisode(index);
	UINFO("Recalled episode %i", index);*/
	return lastActivatedEpisode;
}

void EpisodeLayer::initializeCategories()
{
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it!=m_mapChannel.end() ; it++)
	{
		//load input categories from database
		std::map<int,CategoryARTptr> mapCategory = m_patternRecognizerDao->findAllByChannel((*it).second);
		(*it).second->setMapCategory(mapCategory);

	}
}

bool EpisodeLayer::clearPatternRecognizers()
{
	bool success = true;
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			(*it2).second.reset();
			success &= ((*it2).second.use_count() == 0);
		}
		if(success)
			mapCategory.clear();
	}
	if(success)
		m_patternRecognizerDao->clearTable();

	return success;
}

void EpisodeLayer::notifyLearningModeChanged()
{
	emit learningModeChanged(this->m_learningMode);
}

void EpisodeLayer::printLayerInfo()
{
	ROS_INFO("Display Episode Layer info");
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin(); it != m_mapChannel.end() ; it++)
	{
		//print Channel Description
		ROS_INFO("Channel %i : %s, relevance : %5.2f", 
                 (*it).first,
                 (*it).second->getDescription().c_str(),
                 (*it).second->getRelevance());
		//print input
		(*it).second->printCategories();
	}
}

