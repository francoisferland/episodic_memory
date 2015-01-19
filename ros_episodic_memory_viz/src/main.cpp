/*
 * ros_episodic_memory_visualizer.cpp
 *
 *  Created on: 2012-10-23
 *      Author: frank
 */

#include <QtGui>
#include <myApp.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <boost/thread.hpp>
#include <ros_networkvisualizer.h>
#include <signal.h>

void my_handler(int s){
	QApplication::closeAllWindows();
	QApplication::exit();
}

int main(int argc, char **argv)
{
	MyApp app(argc, argv);

	ros::init(argc,argv,"ros_episodic_memory_viz");
	ros::NodeHandle node;

	// Catch ctrl-c to close the gui
	// (Place this after QApplication's constructor)
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	ROS_NetworkVisualizer w(&node);

	//subscribe to topic to receive changes from the core
	ros::Subscriber updatedWeightSubscriber = node.subscribe("updated_weight",10000,&ROS_NetworkVisualizer::updatedWeightSubscriber,&w);
	ros::Subscriber newCategory = node.subscribe("new_category",1000, &ROS_NetworkVisualizer::newCategorySubscriber, &w);
	ros::Subscriber categoryActivation = node.subscribe("category_activation",1000, &ROS_NetworkVisualizer::updateActivationSubscriber, &w);
	ros::Subscriber newChannel = node.subscribe("new_channel",100, &ROS_NetworkVisualizer::newChannelSubscriber, &w);
	ros::Subscriber anticipatedEvent = node.subscribe("anticipated_event",100, &ROS_NetworkVisualizer::anticipatedEventSubscriber, &w);
	ros::Subscriber learningModeChange = node.subscribe("change_learning_mode",5, &ROS_NetworkVisualizer::learningModeChangeSubscriber,&w);
	//subscribe to emotions
	ros::Subscriber emotionSub = node.subscribe("emotions",100,&ROS_NetworkVisualizer::callbackEmotion, &w);

	//publisher
	ros::Publisher pubInputData = node.advertise<ros_episodic_memory::inputData>("input_data",1000,false);
	w.setInputDataPublisher(&pubInputData);
	ros::Publisher pubLearningMode = node.advertise<ros_episodic_memory::learningMode>("force_change_learning_mode",5,false);
	w.setLearningModePublisher(&pubLearningMode);

	w.show();
	app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));

	ros::AsyncSpinner spinner(0);
	spinner.start();

	ros::Duration(0.5).sleep();

	//use service provided by ros_episodic_memory node to initialize visualizer
	ros::ServiceClient client = node.serviceClient<std_srvs::Empty>("get_initial_values");
	std_srvs::Empty empty;
	if(!client.call(empty))
	{
		ROS_WARN("Warning, call to service provided by ros_episodic_memory returns nothing");
	}

	int result = app.exec();
	spinner.stop();
	w.joinBackgroundThread();

	return result;
}
