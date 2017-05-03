#ifndef PATTERNRECOGNIZEROBJ_H_
#define PATTERNRECOGNIZEROBJ_H_

#include <iostream>
#include <boost/thread.hpp>
#include "categoryART.h"
#include <ros/ros.h>

class PatternRecognizerObj;
typedef boost::shared_ptr<PatternRecognizerObj> PatternRecognizerObjPtr;

/**
 * This class represent an object of the Input Table in the database
 */
class PatternRecognizerObj : public CategoryART
{
public:
	/**
	 * default Constructor
	 */
	PatternRecognizerObj();
	/**
	 * Constructor
	 * @param number: id number in the layer
	 * @param channel: channel id
	 * @param desc: a textual description of the input
	 */
	PatternRecognizerObj(int uid, int indexPatternRecognizer, float vigilance, float learningRate);
	virtual ~PatternRecognizerObj(){}

	float getVigilance() const
	{
		return m_vigilance;
	}
	void setVigilance(float vigilance)
	{
		if(vigilance != m_vigilance)
			isUpdated_ = true;
		else
			isUpdated_ = false;
		this->m_vigilance = vigilance;
	}

	float getLearningRate() const
	{
		return m_learningRate;
	}
	void setLearningRate(float learningRate)
	{
		if(learningRate != this->m_learningRate)
			isUpdated_ = true;
		else
			isUpdated_ = false;
		this->m_learningRate = learningRate;
	}

	bool isUpdated() const {
		return isUpdated_;
	}

	float computeLearningRate(float currentEmotionIntensity,bool isBeforeLearning);
	float computeVigilance(float currentEmotionIntensity);

private:

	float m_vigilance;
	float m_learningRate;

	bool isUpdated_;

};


#endif /* PATTERNRECOGNIZEROBJ_H_ */
