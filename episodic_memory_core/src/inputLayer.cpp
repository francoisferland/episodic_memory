#include "inputLayer.h"

InputLayer::InputLayer():LayerART()
{
	m_layerId = INPUT_LAYER;
}

InputLayer::~InputLayer()
{
}

void InputLayer::initializeCategories()
{
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it!=m_mapChannel.end() ; it++)
	{
		//load input categories from database
		std::map<int,CategoryARTptr> mapCategory = m_inputDao->findAllByChannel((*it).second);
		(*it).second->setMapCategory(mapCategory);

		//link each input categories to its complement
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			InputObjPtr input = boost::dynamic_pointer_cast<InputObj>((*it2).second);
			if(input && input->getDescription().find_first_of('~') == std::string::npos)
			{
				//get the next input by unique index
				CategoryARTptr tempInputComplement = mapCategory[input->getUid()+1];
				InputObjPtr inputComplement = boost::dynamic_pointer_cast<InputObj>(tempInputComplement);
				input->setComplementInput(inputComplement);
			}
		}

	}
}

CategoryARTptr InputLayer::resonance()
{
	CategoryARTptr upperCategory;

	//verify if the upper layer is empty. if so learn the pattern
	if(m_upperLayer->getListCategory().size() == 0)
	{
		m_upperLayer->addChannel("eventLayerChannel",1);
		upperCategory = m_upperLayer->addCategory();
		//create weightObj between each input node and the new category
		createWeightPattern(upperCategory,true);

	}
	else
	{
		upperCategory = recognition();
		//learn the pattern with existing patternRecognizer or new one
		m_upperLayer->learnPattern(upperCategory);
		upperCategory->setActivationValue(1);
		upperCategory->setActivatedTime();
		//decrease activation in upper layer, exept the latest activated category
		m_upperLayer->decreaseActivation(upperCategory);
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

InputObjPtr InputLayer::addNewInput(std::string description, std::string channelDescription)
{
	InputObjPtr inputObj;
	InputObjPtr inputObjComplement;

	//find the channel where to add this new category
	ChannelARTptr channel = getChannelByDescription(channelDescription);
	if(channel)
	{
		inputObj = InputObjPtr(new InputObj());
		inputObj->setChannel(channel);
		inputObj->setDescription(description);
		inputObj->setIndexCategory(generateIndexCategory());
		//updateDatabase
		if(updateInputDatabase(inputObj))
		{
			channel->addInput(inputObj);

			//add complement
			inputObjComplement = InputObjPtr(new InputObj());
			inputObjComplement->setChannel(channel);
			inputObjComplement->setDescription("~"+description);
			inputObjComplement->setIndexCategory(generateIndexCategory());
			//updateDatabase
			updateInputDatabase(inputObjComplement);

			inputObj->setComplementInput(inputObjComplement);
			channel->addInput(inputObjComplement);
		}
		else
			inputObj.reset();

	}
	//add default weight for the existing upper layer pattern recognizer
	if(m_upperLayer->getListCategory().size())
	{
		std::map<int,ChannelARTptr> mapChannelUpperLayer = m_upperLayer->getMapChannel();
		for(std::map<int,ChannelARTptr>::iterator it = mapChannelUpperLayer.begin() ; it != mapChannelUpperLayer.end() ; it++)
		{
			std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
			for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
			{
				//the input is set to zero and its complement to one, because the patternRecognizer never saw those inputs
				WeightObjPtr weight1 = addWeight(inputObj,(*it2).second,0);
				WeightObjPtr weight2 = addWeight(inputObjComplement,(*it2).second,1);
			}
		}
	}

	//notify new inputs
	if(inputObj)
	{
		notifyNewCategory(inputObj);
	}
	if(inputObjComplement)
	{
		notifyNewCategory(inputObjComplement);
	}

	return inputObj;
}

int InputLayer::generateIndexCategory()
{
	int index = -1;

	index = this->getListCategory().size();
	index += m_upperLayer->getListCategory().size();
	index += m_upperLayer->getUpperLayer()->getListCategory().size();

	return index;
}

bool InputLayer::updateInputDatabase(InputObjPtr inputObj)
{
	bool success = true;

	//update if the uid is known
	if(inputObj->getUid() != -1)
	{
		success &= m_inputDao->updateObj(inputObj);
	}
	else
	{
		//verify if an input with the same description already exists before inserting
		InputObjPtr tempInput = m_inputDao->findByDescription(inputObj->getDescription());
		if(tempInput)
			success = false;
		tempInput.reset();
		if(success)
		{
			//create a database channel object
			success &= m_inputDao->insertObj(inputObj);
		}
	}

	return success;
}

void InputLayer::printLayerInfo()
{

}

InputObjPtr InputLayer::getCategoryByDescription(std::string description)
{
	InputObjPtr inputObj;
	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			InputObjPtr tempInputObj = boost::dynamic_pointer_cast<InputObj>((*it2).second);
			if(tempInputObj->getDescription().compare(description) == 0)
			{
				inputObj = tempInputObj;
				break;
			}
		}
		if(inputObj)
			break;
	}

	return inputObj;
}

bool InputLayer::clearInputs()
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
			(*it).second->getMapCategories().clear();
	}
	if(success)
		m_inputDao->clearTable();

	return success;
}

bool InputLayer::activateInputs(std::map<std::string,InputROSmsgPtr> mapActiveInput)
{
	bool success = true;
	unsigned int iteration = 0;

	for(std::map<int,ChannelARTptr>::iterator it = m_mapChannel.begin() ; it != m_mapChannel.end() ; it++)
	{
		std::map<int,CategoryARTptr> mapCategory = (*it).second->getMapCategories();
		for(std::map<int,CategoryARTptr>::iterator it2 = mapCategory.begin() ; it2 != mapCategory.end() ; it2++)
		{
			InputObjPtr tempInputObj = boost::dynamic_pointer_cast<InputObj>((*it2).second);
			//skip if complement
			if(!tempInputObj->getComplementInput())
				continue;

			std::string desc = tempInputObj->getDescription();
			if(mapActiveInput.count(desc))
			{
				iteration++;
				float activationValue = mapActiveInput[desc]->getActivValue();
				tempInputObj->setActivationValue(activationValue);
				tempInputObj->setActivatedTime();
				tempInputObj->getComplementInput()->setActivationValue( 1 - activationValue );
			}
			else
			{
				//set to zero and set the complement to 1
				tempInputObj->setActivationValue(0);
				tempInputObj->getComplementInput()->setActivationValue(1);
			}

		}
	}

	if(iteration != mapActiveInput.size())
		success = false;

	//notify the view
	if(success)
	{
		this->notifyActivation(true);
	}

	return success;
}
