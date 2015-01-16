#include "channelART.h"
#include "categoryART.h"
#include "patternRecognizerObj.h"

ChannelART::ChannelART()
{
	setDebugResonance(false);
	m_channelObj = ChannelObjPtr(new ChannelObj);
	this->m_mapCategory.clear();
}

int ChannelART::addInput(boost::shared_ptr<CategoryART> category)
{
	int index = -1;
	if(category != NULL)
	{
		//update the categoryList
		index = category->getUid();
		if(index != -1)
			this->m_mapCategory[index] = category ;
	}

	return index;
}

bool ChannelART::activateInput(int p_index, float p_activationValue)
{
	bool isActivated = false;
	if( (p_activationValue >= 0) && (p_activationValue <=1))
	{
		if(m_mapCategory[p_index])
		{
			m_mapCategory[p_index]->setActivationValue(p_activationValue);
			m_mapCategory[p_index]->setActivatedTime();
			isActivated = true;
		}
	}
	return isActivated;
}

std::map<int,ResonanceInfo> ChannelART::resonance(matrix<float> weightMatrix,std::vector<float> vigilanceVect)
{
	std::map<int,ResonanceInfo> resonanceMap;
	vector<float> vecActivation = getActivationValueVect();

	std::map<int,float> unsortedActivatedCategories = activateCategory(weightMatrix,vecActivation);
	//calculate match for each category activated
	for(std::map<int,float>::iterator it = unsortedActivatedCategories.begin() ; it != unsortedActivatedCategories.end() ; it++)
	{
		//compare to vigilance parameter and build a map for resonant category
		if((*it).second > 0 || this->getRelevance() == 0) //only activated category
		{
			float match = calculateMatch(weightMatrix,(*it).first, vecActivation);
			//get the vigilance specific to the category
			int index = (*it).first;
			float vigilance = vigilanceVect[index];
			//modulate the vigilance in accordance with the channel relevance
			vigilance = modulateVigilanceWithRelevance(vigilance);
			//verify that the match is bigger than the vigilance parameter and fill the map
			if(match >= vigilance)
			{
				ResonanceInfo info = ResonanceInfo((*it).first,match,true);
				resonanceMap.insert(std::pair<int,ResonanceInfo>((*it).first,info));

				//ROS_WARN("channel %s, event %i, match %.2f, vigilance %.2f, resonant %i",this->getDescription().c_str(),index,match,vigilance,1);
			}
			else
			{
				ResonanceInfo info = ResonanceInfo((*it).first,match,false);
				resonanceMap.insert(std::pair<int,ResonanceInfo>((*it).first,info));

				ROS_WARN("channel %s, event %i, match %.2f, vigilance %.2f, resonant %i",this->getDescription().c_str(),index,match,vigilance,0);
			}

		}
		else
		{
			//if the category is not activated, there's no resonance
			ResonanceInfo info = ResonanceInfo((*it).first,0,false);
			resonanceMap.insert(std::pair<int,ResonanceInfo>((*it).first,info));
		}
	}

	return resonanceMap;
}

float ChannelART::modulateVigilanceWithRelevance(float vigilance)
{
	float newVigilance = vigilance * this->getRelevance();

	return newVigilance;
}

std::map<int,float> ChannelART::activateCategory(matrix<float> & weightMatrix, vector<float> p_input)
{
	//Activation[j] = |Input^Weight(j)| / (bias + |Weight(j)|)
	ROS_WARN_COND(debugResonance,"Activation[j] = |Input^Weight(j)| / (bias + |Weight(j)|) ");

	float bias = 0.01;
	int indexRow = 0;
	std::map<int,float> sortedActivatedCategories;

	//loop through all category
	for(matrix<float>::iterator1 it = weightMatrix.begin1() ; it != weightMatrix.end1() ; it++)
	{

		if(debugResonance)
		{
			for(vector<float>::iterator it = p_input.begin() ; it != p_input.end() ; it++)
			{
				float value = (*it);
				printf("%5.2f ",value);
			}
			printf("\n");
		}

		int indexColumn = 0;

		matrix_row<matrix<float> > tempCol(weightMatrix,indexColumn);
		vector<float> weightVect = tempCol;
		if(debugResonance)
		{
			for(vector<float>::iterator it = weightVect.begin() ; it != weightVect.end() ; it++)
			{
				printf("%5.2f ",(*it));
			}
		}
		vector<float> minVect(p_input.size());
		for(matrix<float>::iterator2 it2 = it.begin() ; it2 != it.end() ; ++it2)
		{
			float value = (*it2);
			//if(debugResonance)
			//{
			//	printf("%5.2f ",value);
			//}
			//minimum between input vector and weightVect
			boost::tuple<float const&, float const&> result = boost::minmax(value, p_input[indexColumn]);
			//keep the minimum in a vector
			minVect[indexColumn++] = result.get<0>();
		}

		if(debugResonance)
		{
			printf("\n --------------------------------------\n");
			for(vector<float>::iterator it = minVect.begin() ; it != minVect.end() ; it++)
			{
				float value = (*it);
				printf("%5.2f ",value);
			}
			printf("\n");
		}

		float numerator = norm_2(minVect);
		float denumerator = (bias + norm_2(weightVect));
		float categoryActivation = numerator / denumerator;
		sortedActivatedCategories.insert(std::pair<int,float>((indexRow++), categoryActivation));
		ROS_WARN_COND(debugResonance,"activation[%i] = %5.2f / %5.2f = %5.2f ",indexRow-1,numerator,denumerator,categoryActivation);
	}

	return sortedActivatedCategories;
}



double ChannelART::calculateMatch(matrix<float> & weightMatrix, int categoryIndex, vector<float> p_input)
{
	float match = 0;
	//Match = |Input^WeightVector| / |Input|
	ROS_WARN_COND(debugResonance,"Match = |Input^WeightVector| / |Input| ");
	if(debugResonance)
	{
		for(vector<float>::iterator it = p_input.begin() ; it != p_input.end() ; it++)
		{
			float value = (*it);
			printf("%5.2f ",value);
		}
		printf("\n");
	}
	//weigthVector
	const matrix_row<matrix<float> > categoryWeightVect (weightMatrix,categoryIndex);
	int indexRow = 0;
	vector<float> minVect(p_input.size());
	//Input^WeigthVector
	for(matrix_row<matrix<float> >::const_iterator it = categoryWeightVect.begin() ; it != categoryWeightVect.end() ; ++it)
	{
		//minimum between input vector and weightVect
		float weigthValue = (*it);
		if(debugResonance)
		{
			printf("%5.2f ",weigthValue);
		}
		boost::tuple<float const&, float const&> result = boost::minmax(weigthValue, p_input[indexRow]);
		//keep the minimum in a vector
		minVect[indexRow++] = result.get<0>();
	}

	if(debugResonance)
	{
		printf("\n--------------------------------\n");
		for(vector<float>::iterator it = minVect.begin() ; it!=minVect.end() ; it++)
		{
			float value = (*it);
			printf("%5.1f ",value);
		}
		printf("\n");
	}
	float numerator = norm_2(minVect);
	float denum = norm_2(p_input);
	match = numerator / denum;
	ROS_WARN_COND(debugResonance," %5.2f = %5.2f / %5.2f",match,numerator,denum);
	return match;
}


int ChannelART::getNbCategory()
{
	return this->m_mapCategory.size();
}

std::map<int,boost::shared_ptr<CategoryART> > ChannelART::getMapCategories()
{
	return m_mapCategory;
}

vector<float> ChannelART::getActivationValueVect()
{
	vector<float> vect(getNbCategory());
	int index = 0;
	for(std::map<int,boost::shared_ptr<CategoryART> >::iterator it = m_mapCategory.begin(); it != m_mapCategory.end() ; it++)
	{
		vect[index] = (*it).second->getActivationValue();
		index++;
	}
	return vect;
}

void ChannelART::printCategories()
{
	if(m_mapCategory.size() > 0)
	{
		for(std::map<int,boost::shared_ptr<CategoryART> >::iterator it = m_mapCategory.begin(); it != m_mapCategory.end() ; it++)
		{
			(*it).second->print();
			ROS_INFO("\r\n");
		}
	}
	else
	{
		ROS_INFO("no category\r\n ");
	}
}
