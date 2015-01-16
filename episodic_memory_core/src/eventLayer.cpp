#include "eventLayer.h"

EventLayer::EventLayer():LayerART()
{
	m_layerId = EVENT_LAYER;
}

EventLayer::~EventLayer()
{
}

void EventLayer::initializeCategories()
{
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it!=m_mapChannel.end() ; it++)
	{
		//load input categories from database
		std::map<int,CategoryARTptr> mapCategory = m_patternRecognizerDao->findAllByChannel((*it).second);
		(*it).second->setMapCategory(mapCategory);

	}
}

CategoryARTptr EventLayer::resonance()
{
	CategoryARTptr upperCategory;

	//always clear all activation in the episode layer before resonance
	m_upperLayer->clearActivation();

	if( m_upperLayer->getListCategory().size() == 0 && m_upperLayer->getLearningMode() == LEARNING )
	{
		ChannelARTptr channel = m_upperLayer->addChannel("episodeLayerChannel",1);
		upperCategory = m_upperLayer->addCategory();
		//create weightObj between each input node and the new category
		createWeightPattern(upperCategory,true);

	}
	else
	{
		//try recognizing
		upperCategory = recognition();
		if(m_upperLayer->getLearningMode() == LEARNING)
		{
			m_upperLayer->learnPattern(upperCategory);
		}
		if(upperCategory)
		{
			upperCategory->setActivationValue(1);
			upperCategory->setActivatedTime();
		}
	}

	//notify event activation
	m_upperLayer->notifyActivation(true);

	//publish all the weights
	if(upperCategory)
	{
		notifyUpdatedWeight(upperCategory);
	}

	return upperCategory;
}

bool EventLayer::createWeightPattern(CategoryARTptr & upperLayerCategory, bool fromUpperCategory)
{
	LayerART::createWeightPattern(upperLayerCategory,fromUpperCategory);

	//after the pattern is created, we must update the upperCategory vigilance according to emotion intensity and episode size
	EpisodeLayerPtr episodeLayer = boost::dynamic_pointer_cast<EpisodeLayer>(m_upperLayer);
	episodeLayer->computeVigilance(upperLayerCategory);

	return true;
}

CategoryARTptr EventLayer::addCategory()
{
	CategoryARTptr category;
	ChannelARTptr channel = getChannelByDescription("eventLayerChannel");
	if(channel)
	{
		category = CategoryARTptr(new PatternRecognizerObj());
		PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>(category);
		patternRecognizer->setChannel(channel);
		patternRecognizer->setIndexCategory(generateIndexCategory());

		//set learningRate according to current emotionIntensity
		patternRecognizer->computeLearningRate(currentEmotionIntensity_,false);
		patternRecognizer->setVigilance(0.95);

		//updateDatabase
		if(updatePatternRecognizerDatabase(patternRecognizer))
		{
			channel->addInput(patternRecognizer);
		}
		else
			category.reset();

		//set the max activation value
		category->setActivationValue(1);
		category->setActivatedTime();

		//notify view
		notifyNewCategory(patternRecognizer);
	}

	return category;
}

bool EventLayer::decreaseActivation(CategoryARTptr category)
{
	bool success = true;

	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it!= m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			if((*it2).second != category)
			{
				double lastActivation = (*it2).second->getActivationValue();
				double newActivation = this->getMemoryDecay()->decreaseActivation(lastActivation);
				(*it2).second->setActivationValue(newActivation);
			}
		}
	}

	return success;
}

std::vector<CategoryARTptr> EventLayer::getInputPattern(CategoryARTptr event)
{
	std::vector<CategoryARTptr> inputPattern;

	std::map<int,WeightObjPtr> mapWeight = event->getMapDownWeight();
	for(std::map<int,WeightObjPtr>::iterator it2 = mapWeight.begin() ; it2 != mapWeight.end() ; it2++)
	{
		CategoryARTptr category = (*it2).second->getDownCategoryPtr();
		InputObjPtr inputObj = boost::dynamic_pointer_cast<InputObj>(category);

		if(((*it2).second->getValue() > 0) && inputObj->getComplementInput()) //if there is a complement to this input, it means we are looking at the positive value
		{
			inputPattern.push_back(inputObj);
		}
	}

	return inputPattern;
}

int EventLayer::generateIndexCategory()
{
	int index = -1;

	index = this->getListCategory().size();
	index += m_upperLayer->getListCategory().size();
	index += m_lowerLayer->getListCategory().size();

	return index;
}

void EventLayer::resetVigilance()
{
	for(std::map<int,ChannelARTptr>::iterator it = this->m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			PatternRecognizerObjPtr patternDB = m_patternRecognizerDao->find((*it2).second->getUid());
			if(patternDB)
			{
				PatternRecognizerObjPtr pattern = boost::dynamic_pointer_cast<PatternRecognizerObj>((*it2).second);
				pattern->setVigilance(patternDB->getVigilance());
			}
		}
	}
}

bool EventLayer::updatePatternRecognizerDatabase(PatternRecognizerObjPtr patternRecognizerObj)
{
	bool success = true;

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

bool EventLayer::clearPatternRecognizers()
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

void EventLayer::printLayerInfo()
{

}

