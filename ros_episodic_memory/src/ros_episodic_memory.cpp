#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

//#include <utilite/UtiLite.h>
#include <boost/thread.hpp>

#include <episodic_memory_core/config.h>

#include "episodicMemoryWrapperROS.h"
#include "ros_episodic_memory/updatedWeight.h"
#include "ros_episodic_memory/newCategory.h"
#include "ros_episodic_memory/newChannel.h"
#include "ros_episodic_memory/recalledEpisode.h"
#include "ros_episodic_memory/categoryActivation.h"
#include "perception_context_msgs/egosphereContent.h"
#include "episodic_memory_core/episodeLayer.h"

void connectQt(EM_CORE::EpisodicMemoryCorePtr emPtr, EpisodicMemoryWrapperROS * wrapper)
{

	QObject::connect(emPtr->getInputLayer().get(),SIGNAL(newChannelSignal(int,QString,float,int)),
			wrapper, SLOT(pubNewChannel(int,QString,float,int)));
	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(newChannelSignal(int,QString,float,int)),
			wrapper, SLOT(pubNewChannel(int,QString,float,int)));
	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(newChannelSignal(int,QString,float,int)),
			wrapper, SLOT(pubNewChannel(int,QString,float,int)));

	QObject::connect(emPtr->getInputLayer().get(),SIGNAL(newCategorySignal(int,int,QString,float,float,int)),
			wrapper, SLOT(pubNewCategory(int,int,QString,float,float,int)));
	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(newCategorySignal(int,int,QString,float,float,int)),
			wrapper, SLOT(pubNewCategory(int,int,QString,float,float,int)));
	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(newCategorySignal(int,int,QString,float,float,int)),
			wrapper, SLOT(pubNewCategory(int,int,QString,float,float,int)));

	QObject::connect(emPtr->getInputLayer().get(),SIGNAL(updateActivationSignal(int,int,float,int)),
			wrapper, SLOT(pubUpdateActivation(int, int, float, int)) );
	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(updateActivationSignal(int,int,float,int)),
			wrapper, SLOT(pubUpdateActivation(int, int, float, int)) );
	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(updateActivationSignal(int,int,float,int)),
			wrapper, SLOT(pubUpdateActivation(int, int, float, int)) );

	QObject::connect(emPtr->getInputLayer().get(),SIGNAL(updateWeightSignal(int, int, int, int, float)),
			wrapper, SLOT(pubUpdateWeight(int, int, int, int, float)));
	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(updateWeightSignal(int, int, int, int, float)),
			wrapper, SLOT(pubUpdateWeight(int, int, int, int, float)));
	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(updateWeightSignal(int, int, int, int, float)),
			wrapper, SLOT(pubUpdateWeight(int, int, int, int, float)));

	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(anticipatedEventSignal(int, float, float, bool)),
			wrapper,SLOT(pubAnticipatedEvent(int, float, float, bool)), Qt::UniqueConnection);
	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(anticipatedEventSignal(int, float, float, bool)),
			wrapper,SLOT(pubAnticipatedEvent(int, float, float, bool)), Qt::UniqueConnection);

	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(episodeRecognised( std::vector<CategoryARTptr> )),
			wrapper, SLOT(pubRecognizedEpisode( std::vector<CategoryARTptr> )));

	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(learningModeChanged(EM_CORE::LEARNING_MODE)),
			wrapper, SLOT(pubLearningModeChanged(EM_CORE::LEARNING_MODE)));
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "ros_episodic_memory");

	ros::NodeHandle n;

	// kTypeFile or kTypeNoLog (nothing is logged).
	// ULogger::setType(ULogger::kTypeConsole);
	// Set the logger severity level (kDebug, kInfo, kWarning, kError).
	// All log entries under the severity level are not logged. Here,
	// only debug messages are not logged.
	//ULogger::setLevel(ULogger::kWarning);
	//ULogger::setPrintWhere(false);
	//ULogger::setPrintTime(false);
	//ULogger::setPrintLevel(false);
	//ULogger::setPrintEndline(false);
	//ULogger::setPrintColored(true);


	//start episodicMemory-core
	const char* databasePath = EPISODIC_MEMORY_DATABASE_PATH;

	EM_CORE::EpisodicMemoryCorePtr ep = EM_CORE::EpisodicMemoryCorePtr(new EM_CORE::EpisodicMemoryCore(&databasePath));
	ep->initialize();

	//intialize wrapper
	EpisodicMemoryWrapperROS emWrapper(ep);

	connectQt(ep, &emWrapper);

	//Subsribe to short term memory
	ros::Subscriber sub = n.subscribe("stm_content", 100, &EpisodicMemoryWrapperROS::callbackShortTermMemory, &emWrapper);
	//subscribe to data coming for the gui
	ros::Subscriber inputDataSub = n.subscribe("input_data",100,&EpisodicMemoryWrapperROS::receiveInputData,&emWrapper);
	//subscribe to emotions
	ros::Subscriber emotionSub = n.subscribe("emotions",100,&EpisodicMemoryWrapperROS::callbackEmotion, &emWrapper);
	//subscride to forced change learning mode
	ros::Subscriber changeLearningModeSub = n.subscribe("force_change_learning_mode",5,&EpisodicMemoryWrapperROS::callbackLearningMode,&emWrapper);

	//provide service for resetting the memory
	ros::ServiceServer clearMemoryService = n.advertiseService("clear_em_memory", &EpisodicMemoryWrapperROS::clearEmMemory, &emWrapper);
	//provide service for retrieving the initial values of the art network
	ros::ServiceServer getInitialValuesService = n.advertiseService("get_initial_values",&EpisodicMemoryWrapperROS::getInitialValues,&emWrapper);

	//provide a service to recall the last activated episode without altering the memory
	//ros::ServiceServer recallLastEpisodeService = n.advertiseService("recall_last_episode",&EpisodicMemoryWrapperROS::recallLastEpisode,&emWrapper);

	//faire un publish message pour lorsquun evenement est process, publier les poids qui ont changé seulement
	ros::Publisher pubUpdatedWeight = n.advertise<ros_episodic_memory::updatedWeight>("updated_weight",10000,false);
	emWrapper.setUpdatedWeightPublisher(&pubUpdatedWeight);
	//publier les nouvelles categories de noeuds
	ros::Publisher pubNewCategory = n.advertise<ros_episodic_memory::newCategory>("new_category",100,false);
	emWrapper.setNewCategoryPublisher(&pubNewCategory);
	//publier les activation de categories
	ros::Publisher pubCategoryActivation = n.advertise<ros_episodic_memory::categoryActivation>("category_activation",1000,false);
	emWrapper.setCategoryActivationPublisher(&pubCategoryActivation);
	//publier les nouveaux channels
	ros::Publisher pubNewChannel = n.advertise<ros_episodic_memory::newChannel>("new_channel",100,false);
	emWrapper.setNewChannelPublisher(&pubNewChannel);
	//publier les événements anticipés
	ros::Publisher pubAnticipatedEvent = n.advertise<ros_episodic_memory::anticipatedEvent>("anticipated_event",10,false);
	emWrapper.setAnticipatedEventPublisher(&pubAnticipatedEvent);

	ros::Publisher pubLearningMode = n.advertise<ros_episodic_memory::learningMode>("change_learning_mode",2,true);
	emWrapper.setLearningModePublisher(&pubLearningMode);

	//publier les recalled episode
	ros::Publisher pubRecalledEpisode = n.advertise<ros_episodic_memory::recalledEpisode>("recalled_episode",100,false);
	emWrapper.setRecallEpisodePublisher(&pubRecalledEpisode);

	ros::ServiceServer vigilanceService = n.advertiseService("set_vigilance_parameter",&EpisodicMemoryWrapperROS::vigilanceService,&emWrapper);
	ros::ServiceServer learningRateService = n.advertiseService("set_learningRate_parameter",&EpisodicMemoryWrapperROS::learningRateService,&emWrapper);
	ros::ServiceServer relevanceService = n.advertiseService("set_relevance_parameter",&EpisodicMemoryWrapperROS::relevanceService,&emWrapper);
	ros::ServiceServer clearAllActivation = n.advertiseService("clear_all_activation",&EpisodicMemoryWrapperROS::clearActivationService,&emWrapper);

	ROS_INFO("*** node ros_episodic_memory v3 ***");

	ros::spin();

	return 0;

}
