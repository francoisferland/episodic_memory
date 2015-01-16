/*
 * inputData.h
 *
 *  Created on: 2013-06-12
 *      Author: frank
 */

#ifndef INPUTDATA_H_
#define INPUTDATA_H_

namespace EM_CORE
{
class InputData;
typedef boost::shared_ptr<InputData> InputDataPtr;

class InputData
{
public:
	InputData(std::string description,std::string channelDescription,float activationValue,int matchRelevance)
	{description_=description;channelDescription_=channelDescription;activationValue_=activationValue;matchRelevance_=matchRelevance;}

	std::string getDesc(){return description_;}
	std::string getChannelDesc(){return channelDescription_;}
	float getActivValue(){return activationValue_;}
	int getMatchRelev(){return matchRelevance_;}

private:
		std::string description_;
		std::string channelDescription_;
		float activationValue_;
		int matchRelevance_;
};
}

#endif /* INPUTDATA_H_ */
