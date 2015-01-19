/*
 * stmObjects.cpp
 *
 *  Created on: 2013-03-28
 *      Author: frank
 */

#include <ros/ros.h>
#include "egosphere_ros/egosphere.hpp"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include "stmInterface.hpp"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

using namespace egosphere_ros;
using namespace cv;

const float secondBetweenObjectDetect = 4;

class StmObjects : public StmInterface
{
public:
	StmObjects(EgosPtr, ros::NodeHandle nh, int duration);
	~StmObjects();

	void updateData(const std_msgs::Float32MultiArrayConstPtr objects);

	bool verifyPosition(std::vector<Point2f> objectCorners);

	bool verifyArea(std::vector<Point2f> objectCorners);

	void imageCallback(const sensor_msgs::ImageConstPtr& msg);


private:
	ros::Publisher pubVoice;
	ros::Publisher pubObjectImage;
	image_transport::ImageTransport it_;
	image_transport::Subscriber subImage;

	int cameraWidth;
	int cameraHeight;

	cv_bridge::CvImagePtr cv_ptr;

	bool isSubscriberObjectImage;
};

