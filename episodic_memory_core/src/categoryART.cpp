#include "categoryART.h"
#include "channelART.h"
#include "weightObj.h"

CategoryART::CategoryART()
{
	m_uid = -1;
	m_indexCategory = -1;
	m_activationValue = 0;
	m_lastTimeActivated = not_a_date_time;
	m_isActivationChange = true;
}

CategoryART::CategoryART(int uid)
{
	m_uid = uid;
	m_indexCategory = 0;
	m_activationValue = 0;
	m_lastTimeActivated = not_a_date_time;
	m_isActivationChange = true;
}

CategoryART::~CategoryART()
{
	m_channelPtr.reset();
}

void CategoryART::print()
{
	UINFO("    %i, Activation value : %5.4f" ,getUid() , getActivationValue());
}

void CategoryART::setActivatedTime()
{
	m_lastTimeActivated = ptime(second_clock::local_time());
}

time_duration CategoryART::getElapseTime()
{
	time_duration elapseTime ;

	elapseTime = ptime(second_clock::local_time()) - m_lastTimeActivated;

	return elapseTime;
}

void CategoryART::setActivationValue(double p_activationValue)
{
	if(p_activationValue != m_activationValue)
		m_isActivationChange = true;

	m_activationValue = p_activationValue;
}

double CategoryART::getActivationValue()
{
	return m_activationValue;
}

void CategoryART::clearOnChange()
{
	m_isActivationChange = false;
}

ChannelARTptr CategoryART::getChannel() const
{
	return m_channelPtr;
}

void CategoryART::setChannel(ChannelARTptr channelPtr)
{
	m_channelPtr = channelPtr;
}

void CategoryART::addUpWeight(WeightObjPtr weight)
{
	m_mapUpWeight[weight->getUID()] = weight;
}

void CategoryART::addDownWeight(WeightObjPtr weight)
{
	m_mapDownWeight[weight->getUID()] = weight;
}

WeightObjPtr CategoryART::findWeightByConnection(CategoryARTptr category)
{
	WeightObjPtr weight;

	for(std::map<int,WeightObjPtr>::iterator it = m_mapUpWeight.begin() ; it != m_mapUpWeight.end() ; it++)
	{
		if((*it).second->getUpCategoryPtr() == category)
		{
			weight = (*it).second;
			break;
		}
	}
	if(!weight)
	{
		for(std::map<int,WeightObjPtr>::iterator it = m_mapDownWeight.begin() ; it != m_mapDownWeight.end() ; it++)
		{
			if((*it).second->getDownCategoryPtr() == category)
			{
				weight = (*it).second;
				break;
			}
		}
	}
	return weight;
}

/*
 * minimum vigilance is the lowest activated category
 */
float CategoryART::getMinimumVigilance(int & nbActiveInput)
{
	float minVigilance = 1;
	nbActiveInput = 0;

	std::map<int,WeightObjPtr > downWeight = this->getMapDownWeight();
	for(std::map<int,WeightObjPtr >::iterator it = downWeight.begin() ; it != downWeight.end() ; it++)
	{
		if((*it).second->getValue() > 0)
		{
			nbActiveInput++;
			if((*it).second->getValue() < minVigilance)
				minVigilance = (*it).second->getValue();
		}
	}

	if(minVigilance < MIN_VIGILANCE)
		minVigilance = MIN_VIGILANCE;

	//minimum vigilance is a little bit bigger than the smallest activated category,
	//so the activation of the first category only wont provoke a recognition
	return minVigilance * 1.01;
}
