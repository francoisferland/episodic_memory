/*
 * stmObjects.cpp
 *
 *  Created on: 2013-03-28
 *      Author: frank
 */

#include "stmObjects.h"

StmObjects::StmObjects(EgosPtr egos, ros::NodeHandle nh, int duration):StmInterface(egos),it_(nh)
{
	setDuration(duration);
	cameraWidth = 640;
	cameraHeight = 480;

	pubVoice = nh.advertise<std_msgs::String>("say_fr",1,true);
	pubObjectImage = nh.advertise<sensor_msgs::Image>("objectDetected",1,true);

	isSubscriberObjectImage = false;
}

StmObjects::~StmObjects()
{

}

void StmObjects::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//convert from ros image format to opencv image format
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}

bool StmObjects::verifyPosition(std::vector<Point2f> objectCorners)
{
	if(		objectCorners[0].x > 0 && objectCorners[3].x > 0 &&
			objectCorners[0].y > 0 && objectCorners[1].y > 0 &&
			objectCorners[2].y < cameraHeight && objectCorners[3].y < cameraHeight &&
			objectCorners[1].x < cameraWidth && objectCorners[2].x < cameraWidth )
	{
		return true;
	}
	return false;
}

bool StmObjects::verifyArea(std::vector<Point2f> objectCorners)
{
	//from the point P0, we compute the norm of the vector with each other point
	//each vector must be at least 20 pix
	float norm1squared = pow((objectCorners[0].x - objectCorners[1].x),2) + pow((objectCorners[0].y - objectCorners[1].y),2);
	float norm2squared = pow((objectCorners[0].x - objectCorners[2].x),2) + pow((objectCorners[0].y - objectCorners[2].y),2);
	float norm3squared = pow((objectCorners[0].x - objectCorners[3].x),2) + pow((objectCorners[0].y - objectCorners[3].y),2);
	if(norm1squared > 400 && norm2squared > 400 && norm3squared > 400)
	{
		//we verify that the vector orientation are not all in the same direction, so the shape would be flat
		if((abs(objectCorners[0].x - objectCorners[1].x) > 20 || abs(objectCorners[0].x - objectCorners[2].x) > 20 || abs(objectCorners[0].x - objectCorners[3].x) > 20) &&
				(abs(objectCorners[0].y - objectCorners[1].y) > 20 || abs(objectCorners[0].y - objectCorners[2].y) > 20 || abs(objectCorners[0].y - objectCorners[3].y) > 20)	)
		{
			return true;
		}
	}
	return false;
}

void StmObjects::updateData(const std_msgs::Float32MultiArrayConstPtr objects)
{

	int nbObject = objects->data.size() / 12;
	ROS_DEBUG("%i objects detected, stm",nbObject);

	bool validObject = false;	

	for(int i = 0; i < nbObject ; i++)
	{
		float objectId = objects->data[0 + i*12];

		//first we verify if the 4 corners of the object is in the view
		float width = objects->data[1 + i*12];
		float height = objects->data[2 + i*12];

		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0,0);						obj_corners[1] = cvPoint((int)width,0);
		obj_corners[2] = cvPoint((int)width,(int)height);	obj_corners[3] = cvPoint(0,(int)height);
		std::vector<Point2f> scene_corners(4);

		Mat H = (Mat_<float>(3,3) << objects->data[3 + i*12], objects->data[6 + i*12], objects->data[9 + i*12],
				objects->data[4 + i*12], objects->data[7 + i*12], objects->data[10 + i*12],
				objects->data[5 + i*12], objects->data[8 + i*12], objects->data[11 + i*12]);
		perspectiveTransform( obj_corners, scene_corners, H);

		/*ROS_INFO("object (%5.2f,%5.2f)(%5.2f,%5.2f)(%5.2f,%5.2f)(%5.2f,%5.2f)"
				,scene_corners[0].x,scene_corners[0].y,scene_corners[1].x,scene_corners[1].y
				,scene_corners[2].x,scene_corners[2].y,scene_corners[3].x,scene_corners[3].y);
		 */
		if(verifyPosition(scene_corners) && verifyArea(scene_corners))
		{
			ROS_INFO("Object %2.0f detected in the view" ,objectId);
			validObject = true;
			std::string objectTag = "object";
			std::string objectRedundancyTag = "objectRedundancy";
			std::string objectIdParam = "objectId";

			Info::Itr objectItr = egos_->infoByTag(objectTag);
			Info::Itr objectRedundancyItr = egos_->infoByTag(objectRedundancyTag);

			//each object must appear twice inside an interval before allowing to publish
			Params param = Params().value(objectIdParam,objectId)
													.value("point0x",scene_corners[0].x)
													.value("point0y",scene_corners[0].y)
													.value("point1x",scene_corners[1].x)
													.value("point1y",scene_corners[1].y)
													.value("point2x",scene_corners[2].x)
													.value("point2y",scene_corners[2].y)
													.value("point3x",scene_corners[3].x)
													.value("point3y",scene_corners[3].y);
			if(!objectItr)
			{
				Info::Ptr infoObject = egos_->newInfo(objectTag, param);
				infoObject->cacheLimits(Info::Duration(getDuration()),10);
				setStmChanged(true);
				//the robot identify the object
				std_msgs::String msg;
				std::string s = boost::lexical_cast<std::string>((int)objectId);
				msg.data = "je vois l'objet " + s;
				pubVoice.publish(msg);
			}
			else
			{
				bool isExist = false;
				//verify if the same object Id exist
				for(Info::Itr p = objectItr; p; p++)
				{
					float egoObjectId = p->get<float>(objectIdParam);
					if(egoObjectId == objectId)
					{
						isExist = true;
						int delay = (ros::Time::now().sec - p->getTimeStamp(objectIdParam).sec);
						if(delay >= 1 && delay <= secondBetweenObjectDetect )
						{
							//item is detected twice in the interval
							if(!objectRedundancyItr)
							{
								ROS_DEBUG("add redundancy");
								//associate the param2
								Info::Ptr infoObjectRedundancy = egos_->newInfo(objectRedundancyTag, param);
								infoObjectRedundancy->cacheLimits(Info::Duration(getDuration()),10);
								setStmChanged(true);
							}
							else
							{
								bool isRedundancyExist = false;
								//verify if the redundancy for this object exist
								for(Info::Itr obj_r = objectRedundancyItr; obj_r; obj_r++)
								{
									float egoObjectRedunId = obj_r->get<float>(objectIdParam);
									if(egoObjectRedunId == objectId)
									{
										isRedundancyExist = true;
										obj_r->set( param , ros::Time::now());
										break;
									}
								}
								if(!isRedundancyExist)
								{
									Info::Ptr infoObjectRedundancy = egos_->newInfo(objectRedundancyTag, param);
									infoObjectRedundancy->cacheLimits(Info::Duration(getDuration()),10);
									setStmChanged(true);
								}
							}

						}
						else
						{
							//update the objectId_1
							p->set( param , ros::Time::now());
							p->cacheDurationLimit(Info::Duration(getDuration()));
						}
						break;
					}
				}
				if(!isExist)
				{
					//if the item doesnt exist, add it to the egosphere
					Info::Ptr infoObject = egos_->newInfo(objectTag, param);
					infoObject->cacheLimits(Info::Duration(getDuration()),10);

					//the robot identify the object
					std_msgs::String msg;
					std::string s = boost::lexical_cast<std::string>((int)objectId);
					msg.data = "c'est l'objet " + s;
					pubVoice.publish(msg);
				}
			}

			if(!isSubscriberObjectImage && pubObjectImage.getNumSubscribers() > 0)
			{
				subImage = it_.subscribe("image",1 , &StmObjects::imageCallback,this);
				isSubscriberObjectImage = true;
			}else if(isSubscriberObjectImage && pubObjectImage.getNumSubscribers() == 0)
			{
				subImage.shutdown();
				isSubscriberObjectImage = false;
			}

			if(isSubscriberObjectImage)
			{
				if(cv_ptr)
				{
					cv::Mat img_buffer = cv_ptr->image;
					// Draw lines between the corners
					// (the mapped object in the scene - image_2 )
					cv::line( img_buffer, scene_corners[0] , scene_corners[1] ,
							cv::Scalar(0, 255, 0), 4 );
					cv::line( img_buffer, scene_corners[1] , scene_corners[2] ,
							cv::Scalar( 0, 255, 0), 4 );
					cv::line( img_buffer, scene_corners[2] , scene_corners[3] ,
							cv::Scalar( 0, 255, 0), 4 );
					cv::line( img_buffer, scene_corners[3] , scene_corners[0] ,
							cv::Scalar( 0, 255, 0), 4 );

					cv_ptr->image = img_buffer;
				}
			}
		}
	}
	if(isSubscriberObjectImage)
	{
		if(cv_ptr && validObject)
		{
			pubObjectImage.publish( cv_ptr->toImageMsg() );
			cv_ptr.reset();
		}
	}
}

