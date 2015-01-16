#include "layerART.h"

LayerART::LayerART()
{
	m_upperLayer.reset();
	m_lowerLayer.reset();

	m_layerId = UNDEFINE;
	m_learningMode = RECOGNIZING;
	currentEmotionIntensity_ = 0;
}

std::map<int,CategoryARTptr> LayerART::getListCategory()
{
	std::map<int,CategoryARTptr> listCategory;
	//merge all the category map into one
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();

		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			listCategory[(*it2).first] = (*it2).second;
		}
	}

	return listCategory;
}

ChannelARTptr LayerART::addChannel(std::string p_description, float p_relevance)
{
	//instanciate new channel
	ChannelARTptr newChannel = ChannelARTptr(new ChannelART);
	newChannel->setDescription(p_description);
	newChannel->setRelevance(p_relevance);
	newChannel->setIndexChannel(m_mapChannel.size());
	newChannel->setLayerId(m_layerId);
	//if(this->getLayerId() == EVENT_LAYER)
	//{
	//	newChannel->setDebugResonance(true);
	//}
	//update database
	bool success = updateChannelDatabase(newChannel);

	if(success)
	{
		m_mapChannel[newChannel->getUid()] = newChannel;
		//notify channel
		notifyNewChannel(newChannel);
	}
	else
		newChannel.reset();

	return newChannel;
}

WeightObjPtr LayerART::addWeight(CategoryARTptr downCategory,CategoryARTptr upCategory, float weightValue)
{
	WeightObjPtr weight = WeightObjPtr(new WeightObj);
	weight->setDownCategoryPtr(downCategory);
	weight->setUpCategoryPtr(upCategory);
	weight->setValue(weightValue);

	bool success = updateWeightDatabase(weight);

	if(success)
	{
		m_mapWeight[weight->getUID()] = weight;
		downCategory->addUpWeight(weight);
		upCategory->addDownWeight(weight);
	}
	else
		weight.reset();

	return weight;
}

CategoryARTptr LayerART::recognition()
{
	CategoryARTptr upperCategory;

	std::map< int,ResonanceInfo > resonantCategory;
	//generate the weight matrix
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		int nbColumn = (*it).second->getMapCategories().size();
		int nbRow = m_upperLayer->getListCategory().size();

		matrix<double> weightMatrix = zero_matrix<float>(nbRow,nbColumn);
		fillWeightMatrix(weightMatrix,(*it).second->getMapCategories());
		//try resonance with each channel
		std::map<int,ResonanceInfo> resonantChannel = (*it).second->resonance(weightMatrix,m_upperLayer->getVigilanceVect());
		for(std::map<int,ResonanceInfo >::iterator it2 = resonantChannel.begin() ; it2 != resonantChannel.end() ; it2++)
		{
			int relativeIndexCategory = (*it2).second.relativeIndexCategory;
			bool resonantStateCurrentChannel = (*it2).second.isResonant;
			float channelMatch = (*it2).second.matchValue;

			//ROS_WARN_COND(this->getLayerId() == INPUT_LAYER, "resonance channel %s, event %i, match %.2f, resonant %i",(*it).second->getDescription().c_str(),(*it2).first,channelMatch,resonantStateCurrentChannel);
			//initialize this map cell
			if(resonantCategory.count(relativeIndexCategory) == 0)
			{
				resonantCategory[relativeIndexCategory] = ResonanceInfo(relativeIndexCategory,0,true);
			}

			//calculate the cumulative resonant state for each category
			resonantCategory[relativeIndexCategory].isResonant &= resonantStateCurrentChannel;
			//calculate the cumulative match for each category
			resonantCategory[relativeIndexCategory].matchValue += channelMatch;
		}
	}

	//now choose the resonant category with the biggest match
	float tempMaxMatch = 0;
	int maxRelativeIndexCategory = -1;
	for(std::map<int,ResonanceInfo >::iterator it3 = resonantCategory.begin() ; it3 != resonantCategory.end() ; it3++)
	{
		bool categoryResonantState = (*it3).second.isResonant;
		float categoryMatch = (*it3).second.matchValue;
		int relativeIndexCategory = (*it3).second.relativeIndexCategory;

		ROS_WARN_COND(this->getLayerId() == EVENT_LAYER,"id:%i, match:%5.3f, vigil=%5.2f",relativeIndexCategory,categoryMatch, m_upperLayer->getVigilanceVect().at(relativeIndexCategory));

		//if the category is in resonant state
		if(categoryResonantState)
		{
			if(categoryMatch > tempMaxMatch)
			{
				tempMaxMatch = categoryMatch;

				maxRelativeIndexCategory = relativeIndexCategory;
			}
		}
	}
	ROS_WARN_COND(this->getLayerId() == EVENT_LAYER,"\n");
	std::map<int,ChannelARTptr> mapChannel = m_upperLayer->getMapChannel();
	for(std::map<int,ChannelARTptr>::iterator it = mapChannel.begin() ; it != mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		int tempIndex = 0;
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			if(tempIndex == maxRelativeIndexCategory)
			{
				upperCategory = (*it2).second;
				break;
			}
			tempIndex++;
		}
		if(upperCategory)
			break;
	}
	return upperCategory;
}

bool LayerART::learnPattern(CategoryARTptr & category)
{
	bool success = true;

	if(!category)
	{
		ROS_INFO_COND((getLayerId()==EPISODE_LAYER),"new episode");
		//create a new patternRecognizer to learn the new pattern
		category = this->addCategory();
		//create weightObj between each input node and the new category
		success &= this->m_lowerLayer->createWeightPattern(category,true);
		//create weightObj between each upper category and the new category
		if(m_upperLayer)
			success &= this->m_upperLayer->createWeightPattern(category,false);
	}
	else
	{
		ROS_INFO_COND((getLayerId()==EPISODE_LAYER),"Learn episode");
		PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>(category);
		//each weight is updated with the current activation value relatively to a learning rate parameter (in patternRecognizer obj)
		//modify learning rate before learning according to emotionIntensity
		//if the intensity is high, the lr will increase, else it will decrease
		float learningRate = patternRecognizer->computeLearningRate(currentEmotionIntensity_,true);

		std::map<int,boost::shared_ptr<WeightObj> > mapWeight = patternRecognizer->getMapDownWeight();

		ROS_INFO_COND((getLayerId()==EPISODE_LAYER),"new_weight = (activation * learningRate) + ((1-learningRate)* weight)");
		//iterate through each down weight of the category to learn the input pattern
		for(std::map<int,boost::shared_ptr<WeightObj> >::iterator it = mapWeight.begin() ; it != mapWeight.end() ; ++it)
		{
			WeightObjPtr weight = (*it).second;
			//the weight object links to the lower layer category, which we can retrieved the activation value to learn
			float activationValue = weight->getDownCategoryPtr()->getActivationValue();
			float weightValue = weight->getValue();
			float newWeightValue = (activationValue * learningRate) + ((1-learningRate) * weightValue);

			ROS_INFO_COND((getLayerId()==EPISODE_LAYER),"weight value modified between %i and %i, %5.3f : (%5.3f * %5.2f) + ((1-%5.2f)*%5.2f) ",weight->getUpCategoryPtr()->getIndexCategory(),weight->getDownCategoryPtr()->getIndexCategory(),newWeightValue,activationValue,learningRate,learningRate,weightValue);

			if(newWeightValue < 0.01)
				newWeightValue = 0;

			weight->setValue(newWeightValue);
			if(std::abs(newWeightValue - weightValue ) > 0.001 )
			{
				//update the weight database
				updateWeightDatabase(weight);
			}
		}

		//modify learning rate after learning according to emotionIntensity
		// if the intensity is high, the lr will decreased so further learning will affect less this emotionnaly salient category
		learningRate = patternRecognizer->computeLearningRate(currentEmotionIntensity_,false);
	}

	return success;
}

bool LayerART::createWeightPattern(CategoryARTptr & upperLayerCategory, bool fromUpperCategory)
{
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> categoryMap = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = categoryMap.begin() ; it2 != categoryMap.end() ; it2++)
		{
			if(fromUpperCategory)
				WeightObjPtr weight = addWeight((*it2).second,upperLayerCategory,(*it2).second->getActivationValue());
			else
			{
				if( getLearningMode() == EM_CORE::LEARNING )
					WeightObjPtr weight = addWeight(upperLayerCategory,(*it2).second,(*it2).second->getActivationValue());
				else if( getLearningMode() == EM_CORE::RECOGNIZING )
				{
					//if a new node is created, we need to create weight even if we are not learning because
					//when the learning occurs, we iterate throughout the weight map of the upper layer node
					WeightObjPtr weight = addWeight(upperLayerCategory,(*it2).second,0);
					ROS_INFO("weigth added in recognizing mode %i",weight->getUID());
				}
			}
		}
	}

	return true;
}

bool LayerART::decreaseActivation(CategoryARTptr category)
{
	return false;
}

bool LayerART::fillWeightMatrix(matrix<double> & weightMatrix, std::map<int,CategoryARTptr> mapDownCategory)
{
	bool success = true;

	if(mapDownCategory.size() != weightMatrix.size2())
		success = false;

	int column = 0;
	for(std::map<int,CategoryARTptr>::iterator it = mapDownCategory.begin() ; it != mapDownCategory.end() ; it++)
	{
		int row = 0;
		std::map<int,WeightObjPtr> mapWeightRow = (*it).second->getMapUpWeight();
		success &= mapWeightRow.size() == weightMatrix.size1();
		if(success)
		{
			for(std::map<int,WeightObjPtr>::iterator it2 = mapWeightRow.begin() ; it2 != mapWeightRow.end() ; it2++)
			{
				weightMatrix.insert_element(row,column,(*it2).second->getValue());
				row++;
			}
			column++;
		}
		else break;
	}

	return success;
}

ChannelARTptr LayerART::getChannelByDescription(std::string description)
{
	ChannelARTptr channel;
	//find the channel where the neuron will be activated
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		ChannelARTptr tempChannel = (*it).second;
		if(tempChannel->getDescription().compare(description) == 0)
		{
			channel = tempChannel;
			break;
		}
	}

	return channel;
}

CategoryARTptr LayerART::getCategoryByIndex(int indexCategory)
{
	CategoryARTptr category;

	std::map<int,ChannelARTptr> mapChannelEventLayer =  getMapChannel();
	for(std::map<int,ChannelARTptr>::iterator it = mapChannelEventLayer.begin() ; it != mapChannelEventLayer.end(); it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ;it2++)
		{
			if((*it2).second->getIndexCategory() == indexCategory)
			{
				category= (*it2).second;
				break;
			}
		}
		if(category)
			break;
	}

	return category;
}

CategoryARTptr LayerART::getLatestActivatedCategory(CategoryARTptr upperCategory)
{
	CategoryARTptr latestActivatedCategory;
	int biggestWeightValue = 0;
	std::map<int,WeightObjPtr> mapWeight = upperCategory->getMapDownWeight();
	for(std::map<int,WeightObjPtr>::iterator it = mapWeight.begin() ; it != mapWeight.end() ; it++)
	{
		if((*it).second->getValue() >= biggestWeightValue && (*it).second->getDownCategoryPtr()->getActivationValue() > 0)
		{
			biggestWeightValue = (*it).second->getValue();
			latestActivatedCategory = (*it).second->getDownCategoryPtr();
		}
	}

	return latestActivatedCategory;
}

CategoryARTptr LayerART::getLastActivatedCategory()
{
	CategoryARTptr lastActivatedCategory;

	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();

		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			time_duration lastActivation = (*it2).second->getElapseTime();
			if(lastActivatedCategory && (lastActivation < lastActivatedCategory->getElapseTime()) )
			{
				lastActivatedCategory = (*it2).second;
			}
			else if( !lastActivatedCategory )
			{
				lastActivatedCategory = (*it2).second;
			}
		}
	}

	return lastActivatedCategory;
}

std::vector<float> LayerART::getVigilanceVect()
{
	std::vector<float> vigilanceVect;
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();

		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>((*it2).second);
			if(patternRecognizer)
				vigilanceVect.push_back(patternRecognizer->getVigilance());
		}
	}

	return vigilanceVect;
}

bool LayerART::updateChannelDatabase(ChannelARTptr channel)
{
	bool success = true;

	//update if the uid is known
	if(channel->getChannelObj()->getUid() != -1)
	{
		success &= m_channelDao->updateObj(channel->getChannelObj());
	}
	else
	{
		//verify if a channel with the same description already exists before inserting
		ChannelObjPtr tempChannel = m_channelDao->findByDescription(channel->getDescription());
		if(tempChannel)
			success = false;
		tempChannel.reset();
		if(success)
		{
			//create a database channel object
			success &= m_channelDao->insertObj(channel->getChannelObj());
		}
	}

	return success;
}

bool LayerART::updateWeightDatabase(WeightObjPtr weightObj)
{
	bool success = true;

	//update if the uid is known
	if(weightObj->getUID() != -1)
	{
		success &= m_weightDao->updateObj(weightObj);
	}
	else
	{
		//verify if a weight with the same connection already exists before inserting
		WeightObjPtr tempWeight = m_weightDao->findByConnection(weightObj->getDownCategoryPtr(),weightObj->getUpCategoryPtr());
		if(tempWeight)
			success = false;
		tempWeight.reset();
		if(success)
		{
			//create a database channel object
			success &= m_weightDao->insertObj(weightObj);
		}
	}

	return success;
}

bool LayerART::clearChannels()
{
	bool success = true;
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		(*it).second.reset();
		success &= ((*it).second.use_count() == 0);
	}
	if(success)
	{
		m_mapChannel.clear();
		m_channelDao->clearTable();
	}
	return success;
}

bool LayerART::clearWeights()
{
	bool success = true;
	for(std::map<int,WeightObjPtr>::iterator it = m_mapWeight.begin() ; it != m_mapWeight.end() ; it++)
	{
		(*it).second.reset();
		success &= ((*it).second.use_count() == 0);
	}
	if(success)
	{
		m_mapWeight.clear();
		m_weightDao->clearTable();
	}
	return success;
}

bool LayerART::clearActivation()
{
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategories = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategories.begin() ; it2 != mapCategories.end() ; it2++)
		{
			(*it2).second->setActivationValue(0);
		}
	}
	return true;
}

bool LayerART::isUpperLayer()
{
	return m_upperLayer?true:false;
}

bool LayerART::isLowerLayer()
{
	return m_lowerLayer?true:false;
}

void LayerART::notifyNewChannel(ChannelARTptr channel)
{
	if(m_mapChannel.count(channel->getUid()))
	{
		emit newChannelSignal(channel->getUid(), QString::fromStdString(channel->getDescription()), channel->getRelevance(), m_layerId);
	}
	else
	{
		ROS_WARN("Channel not created in the core before notifying the view\n");
	}
}

void LayerART::notifyNewCategory(InputObjPtr input)
{
	ChannelARTptr channel = input->getChannel();
	if(channel->getMapCategories().count(input->getUid()))
	{
		emit newCategorySignal(input->getIndexCategory(), input->getChannel()->getUid(), QString::fromStdString(input->getDescription()), 0, 0, m_layerId);
	}
	else
	{
		ROS_WARN("Input not created in the core before notifying the view\n");
	}
}

void LayerART::notifyNewCategory(PatternRecognizerObjPtr patternRecognizer)
{
	ChannelARTptr channel = patternRecognizer->getChannel();
	if(channel->getMapCategories().count(patternRecognizer->getUid()))
	{
		emit newCategorySignal(patternRecognizer->getIndexCategory(), patternRecognizer->getChannel()->getUid(), "", patternRecognizer->getVigilance(), patternRecognizer->getLearningRate(), m_layerId);
	}
	else
	{
		ROS_WARN("PAtternRecognizer not created in the core before notifying the view\n");
	}
}

void LayerART::notifyActivation(bool updatedOnly)
{
	//loop through all the category in this layer and emit signal for changed activation value only
	for(std::map<int,ChannelARTptr>::iterator it= this->m_mapChannel.begin() ; it != m_mapChannel.end(); it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			if(!updatedOnly || (*it2).second->isActivationChange())
			{
				float activation = (*it2).second->getActivationValue();
				(*it2).second->clearOnChange();
				emit updateActivationSignal((*it2).second->getIndexCategory(), (*it).first, activation, m_layerId);
			}
		}
	}
}

void LayerART::notifyUpdatedWeight(CategoryARTptr upperCategory)
{
	//publish every bottom weights linked to this category
	std::map<int,WeightObjPtr> mapWeight = upperCategory->getMapDownWeight();
	for(std::map<int,WeightObjPtr>::iterator it = mapWeight.begin() ; it != mapWeight.end() ; it++)
	{
		CategoryARTptr downCategory = (*it).second->getDownCategoryPtr();
		CategoryARTptr upCategory = (*it).second->getUpCategoryPtr();
		//try to cast into input object
		InputObjPtr input = boost::dynamic_pointer_cast<InputObj>(downCategory);
		if(input && upCategory && input->getComplementInput())
		{
			emit updateWeightSignal((*it).second->getUID(),upCategory->getIndexCategory(), downCategory->getIndexCategory(), m_layerId, (*it).second->getValue());
			(*it).second->setIsUpdated(false);
		}
		if(downCategory && (*it).second->isUpdated())
		{
			emit updateWeightSignal((*it).second->getUID(),upperCategory->getIndexCategory(), downCategory->getIndexCategory(), m_layerId, (*it).second->getValue());
			(*it).second->setIsUpdated(false);
		}
	}
}

void LayerART::notifyAnticipatedEvent(PatternRecognizerObjPtr pattern)
{
	emit anticipatedEventSignal(pattern->getIndexCategory(),pattern->getVigilance(), pattern->getLearningRate(), true);
}

void LayerART::notifyAllWeight()
{
	for(std::map<int,WeightObjPtr>::iterator it = m_mapWeight.begin() ; it != m_mapWeight.end() ; it++)
	{
		CategoryARTptr downCategory = (*it).second->getDownCategoryPtr();
		CategoryARTptr upCategory = (*it).second->getUpCategoryPtr();
		//try to cast into input object
		InputObjPtr input = boost::dynamic_pointer_cast<InputObj>(downCategory);
		if(input && upCategory && input->getComplementInput())
			emit updateWeightSignal((*it).second->getUID(),upCategory->getIndexCategory(), downCategory->getIndexCategory(), m_layerId, (*it).second->getValue());
		else if(downCategory && upCategory )
			emit updateWeightSignal((*it).second->getUID(),upCategory->getIndexCategory(), downCategory->getIndexCategory(), m_layerId, (*it).second->getValue());
	}
}

void LayerART::notifyAllCategories()
{
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it!= m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			//verify if it is a patternRecognizer or an input
			PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>((*it2).second);
			InputObjPtr input = boost::dynamic_pointer_cast<InputObj>((*it2).second);
			if(input)
				emit newCategorySignal(input->getIndexCategory(), input->getChannel()->getUid(), QString::fromStdString(input->getDescription()), 0, 0, m_layerId);
			if(patternRecognizer)
				emit newCategorySignal(patternRecognizer->getIndexCategory(), patternRecognizer->getChannel()->getUid(), "", patternRecognizer->getVigilance(), patternRecognizer->getLearningRate(), m_layerId);
		}
	}
}

void LayerART::notifyAllChannels()
{
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it!= m_mapChannel.end() ; it++)
	{
		emit newChannelSignal((*it).second->getUid(),QString::fromStdString((*it).second->getDescription()),(*it).second->getRelevance(), m_layerId);
	}
}

void LayerART::notifyPatternRecognizerParameter()
{
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it!= m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			notifyPatternRecognizerParameter((*it2).second);
		}
	}
}

void LayerART::notifyPatternRecognizerParameter(CategoryARTptr category)
{
	//verify if it is a patternRecognizer or an input
	PatternRecognizerObjPtr patternRecognizer = boost::dynamic_pointer_cast<PatternRecognizerObj>(category);

	if(patternRecognizer && patternRecognizer->isUpdated())
	{
		ROS_INFO_COND(patternRecognizer->getChannel()->getChannelObj()->getLayerId()==EPISODE_LAYER,"notify episode pattern recognizer %i: %5.2f",patternRecognizer->getIndexCategory(),patternRecognizer->getVigilance());
		emit anticipatedEventSignal(patternRecognizer->getIndexCategory(), patternRecognizer->getVigilance(), patternRecognizer->getLearningRate(), false);
	}
}

