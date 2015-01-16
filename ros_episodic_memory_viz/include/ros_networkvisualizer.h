/*
 * ros_networkvisualizer.h
 *
 *  Created on: 2012-10-24
 *      Author: frank
 */

#ifndef ROS_NETWORKVISUALIZER_H_
#define ROS_NETWORKVISUALIZER_H_

#include <QApplication>
#include <networkvisualizer.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "ros_episodic_memory/updatedWeight.h"
#include "ros_episodic_memory/newCategory.h"
#include "ros_episodic_memory/categoryActivation.h"
#include "ros_episodic_memory/newChannel.h"
#include "ros_episodic_memory/anticipatedEvent.h"
#include "ros_episodic_memory/setVigilanceParameter.h"
#include "ros_episodic_memory/setLearningRateParameter.h"
#include "ros_episodic_memory/setRelevanceParameter.h"
#include "ros_episodic_memory/inputData.h"
#include "ros_episodic_memory/learningMode.h"
#include "hbba_msgs/EmotionIntensities.h"


/**
 * This function manage signal produce by the episodic memory application and adjust the view accordingly
 * It is also the main window program
 */
class ROS_NetworkVisualizer : public NetworkVisualizer
{
	Q_OBJECT

public:

	/**
	 * Default Constructor
	 */
	ROS_NetworkVisualizer(ros::NodeHandle* p_nodeHandle, QWidget *parent = 0);
	/**
	 * destructor
	 */
	~ROS_NetworkVisualizer();

	//subscriber function
	void updatedWeightSubscriber(const ros_episodic_memory::updatedWeight& message);
	void newCategorySubscriber(const ros_episodic_memory::newCategory& message);
	void updateActivationSubscriber(const ros_episodic_memory::categoryActivation& message);
	void newChannelSubscriber(const ros_episodic_memory::newChannel& message);
	void anticipatedEventSubscriber(const ros_episodic_memory::anticipatedEvent& message);
	//
	void setInputDataPublisher(ros::Publisher* p){this->inputDataPublisher = p;}
	void setLearningModePublisher(ros::Publisher* p){this->learningModePublisher = p;}

	void learningModeChangeSubscriber(const ros_episodic_memory::learningMode& msg);

	void callbackEmotion(const hbba_msgs::EmotionIntensitiesPtr& message);

public slots:

void onMenuResetNetwork();
int onChangeVigilance();
int onChangeLearningRate();
int onChangeRelevance(double relevance);

void onPublishLearningMode(EM_CORE::LEARNING_MODE);
void onPublishInputData(std::map<std::string,EM_CORE::InputROSmsgPtr> inputData);
void onPublishClearActivation();

signals:
	void newChannel(int, QString, float, int);
	void newCategory(int index, int channelId, QString description, float vigilance, float learningRate, int layerId);
	void updateWeight(int id, int indexTopCategory, int indexBottomLayer, int idLayer, float value);
	void updateActivation(int index, int idChannel, float value, int idLayer);
	void anticipatedEventSignal(int index, float newVigilance, float newLearningRate, bool anticipated);

private:

	ros::NodeHandle* nodeHandle;
	ros::Publisher* inputDataPublisher;
	ros::Publisher* learningModePublisher;
	int newWeight;

};

#endif /* ROS_NETWORKVISUALIZER_H_ */
