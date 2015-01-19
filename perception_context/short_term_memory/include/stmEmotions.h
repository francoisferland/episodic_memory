#include <ros/ros.h>
#include "stmInterface.hpp"
#include <hbba_msgs/EmotionIntensities.h>

class StmEmotion : public StmInterface
{
public:
	StmEmotion(EgosPtr, int duration);
	~StmEmotion(){;}
	void updateData(const hbba_msgs::EmotionIntensitiesConstPtr emotions);


};

