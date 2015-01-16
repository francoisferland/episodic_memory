#include "patternRecognizerObj.h"

PatternRecognizerObj::PatternRecognizerObj() : CategoryART()
{
	m_vigilance = DEFAULT_VIGILANCE;
	m_learningRate = DEFAULT_LEARNING_RATE;
	isUpdated_ = false;
}

PatternRecognizerObj::PatternRecognizerObj(int uid, int indexPatternRecognizer, float vigilance, float learningRate) : CategoryART(uid)
{
	this->m_vigilance = vigilance;
	this->m_learningRate = learningRate;
	setIndexCategory(indexPatternRecognizer);
	isUpdated_ = false;
}

/**
 * the purpose of this function is to lower the learning rate when the emotion intensity is low. When an
 * event is not emotionally salient, we dont need to keep all the details in memory, so we lower the learning rate.
 */
float PatternRecognizerObj::computeLearningRate(float currentEmotionIntensity, bool isBeforeLearning)
{
	float maxDecrement = this->m_learningRate * 0.25; //the learning rate can be modify by up to 25% each time

	float newLearningRate = 0;
	if(isBeforeLearning)
	{
		newLearningRate = this->m_learningRate - ( maxDecrement * (0.5 - currentEmotionIntensity) );
		//ROS_INFO_NAMED("LearningRate","idx:%i lr = %5.2f = %5.2f - ( %5.1f * (0.5 - %5.2f))\n",this->getIndexCategory(),newLearningRate , this->m_learningRate, maxDecrement, currentEmotionIntensity);
	}
	else
	{ 	maxDecrement*=1.5;
		newLearningRate = this->m_learningRate + ( maxDecrement * (0.5 - currentEmotionIntensity) );
		//ROS_INFO_NAMED("LearningRate","idx:%i lr = %5.2f = %5.2f + ( %5.1f * (0.5 - %5.2f))\n",this->getIndexCategory(),newLearningRate , this->m_learningRate, maxDecrement, currentEmotionIntensity);
	}

	if(newLearningRate > MAX_LEARNING_RATE)
		newLearningRate = MAX_LEARNING_RATE;
	else if(newLearningRate < MIN_LEARNIN_RATE)
		newLearningRate = MIN_LEARNIN_RATE;

	this->setLearningRate(newLearningRate);

	return newLearningRate;
}

float PatternRecognizerObj::computeVigilance(float currentEmotionIntensity)
{
	float maxIncrement = this->getVigilance() * 0.2;
	//float adjustedDefaultVigilance = 0;
	int sizeEpisode = 0;
	float minVigilance = getMinimumVigilance(sizeEpisode);
	//adjust the vigilance according to input vector size
	//adjustedDefaultVigilance = 1 - (((float)sizeEpisode) / 5 / 10);

	float newVigilance = this->getVigilance() - ( maxIncrement * ( currentEmotionIntensity - 0.5 ) );

	if(newVigilance > MAX_VIGILANCE)
		newVigilance = MAX_VIGILANCE;
	else if(newVigilance < minVigilance){
		newVigilance = minVigilance;
		ROS_INFO("vigilance too low, set to minVigilance");
	}

	ROS_INFO_NAMED("Vigilance","idx:%i vig = %5.2f = %5.2f - ( %5.1f * ( %5.2f - 0.5 )) ; min = %.1f \n",this->getIndexCategory(),newVigilance ,this->getVigilance(), maxIncrement, currentEmotionIntensity,minVigilance);
	this->setVigilance(newVigilance);

	return newVigilance;
}
