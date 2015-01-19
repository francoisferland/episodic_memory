#include "myApp.h"
#include <networkvisualizer.h>
#include <episodicMemoryCore.h>
#include <config.h>

using namespace EM_CORE;

void connectQt(EpisodicMemoryCorePtr emPtr, NetworkVisualizer * viz)
{
	QObject::connect(emPtr->getInputLayer().get(),SIGNAL(newCategorySignal(int,int,QString,float,float,int)),
			viz, SLOT(onNewCategory(int,int,QString,float,float,int)));
	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(newCategorySignal(int,int,QString,float,float,int)),
			viz, SLOT(onNewCategory(int,int,QString,float,float,int)));
	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(newCategorySignal(int,int,QString,float,float,int)),
			viz, SLOT(onNewCategory(int,int,QString,float,float,int)));

	QObject::connect(emPtr->getInputLayer().get(),SIGNAL(updateActivationSignal(int,int,float,int)),
			viz, SLOT(onUpdateActivation(int, int, float, int)) );
	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(updateActivationSignal(int,int,float,int)),
			viz, SLOT(onUpdateActivation(int, int, float, int)) );
	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(updateActivationSignal(int,int,float,int)),
			viz, SLOT(onUpdateActivation(int, int, float, int)) );

	QObject::connect(viz, SIGNAL(sendInputData(std::map<std::string,EM_CORE::InputROSmsgPtr>)),
			emPtr.get(),SLOT(onReceiveData(std::map<std::string,EM_CORE::InputROSmsgPtr>)) );

	QObject::connect(viz, SIGNAL(resetNetworkSignal()),
			emPtr.get(),SLOT(clearEpisodicMemory()) );

	QObject::connect(viz, SIGNAL(clearActivationSignal()),
					emPtr.get(), SLOT(clearActivation()));

	QObject::connect(viz, SIGNAL(changeVigilanceSignal(int, float)),
					emPtr.get(),SLOT(setVigilance(int, float)), Qt::UniqueConnection);

	QObject::connect(viz, SIGNAL(changeLearningRateSignal(int, float)),
					emPtr.get(),SLOT(setLearningRate(int, float)), Qt::UniqueConnection);

	QObject::connect(viz, SIGNAL(changeRelevanceSignal(int, float)),
					emPtr.get(),SLOT(setRelevance(int, float)), Qt::UniqueConnection);

	QObject::connect(viz, SIGNAL(toggleLearningMode(EM_CORE::LEARNING_MODE)),
					emPtr.get(), SLOT(setLearningRateMode(EM_CORE::LEARNING_MODE)));

	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(anticipatedEventSignal(int, float,float, bool)),
					viz,SLOT(onAnticipatedEvent(int, float,float, bool)), Qt::UniqueConnection);

	QObject::connect(emPtr->getInputLayer().get(),SIGNAL(updateWeightSignal(int, int, int, int, float)),
					viz, SLOT(onUpdateWeight(int, int, int, int, float)));
	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(updateWeightSignal(int, int, int, int, float)),
					viz, SLOT(onUpdateWeight(int, int , int , int , float )));
	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(updateWeightSignal(int, int, int, int, float)),
					viz, SLOT(onUpdateWeight(int, int, int, int, float)));

	QObject::connect(emPtr->getInputLayer().get(),SIGNAL(newChannelSignal(int,QString,float,int)),
					viz, SLOT(onNewChannel(int,QString,float,int)),Qt::UniqueConnection);
	QObject::connect(emPtr->getEventLayer().get(),SIGNAL(newChannelSignal(int,QString,float,int)),
					viz, SLOT(onNewChannel(int,QString,float,int)),Qt::UniqueConnection);
	QObject::connect(emPtr->getEpisodeLayer().get(),SIGNAL(newChannelSignal(int,QString,float,int)),
					viz, SLOT(onNewChannel(int,QString,float,int)),Qt::UniqueConnection);

	QObject::connect(emPtr->getEpisodeLayer().get(), SIGNAL(learningModeChanged(EM_CORE::LEARNING_MODE)),
					viz, SLOT(onChangeLearningMode(EM_CORE::LEARNING_MODE)));

}

int main(int argc, char *argv[])
{
	const char* databasePath = EPISODIC_MEMORY_DATABASE_PATH;

	EpisodicMemoryCorePtr ep = EpisodicMemoryCorePtr(new EpisodicMemoryCore(&databasePath));
	ep->initialize();

	MyApp a(argc, argv);
	NetworkVisualizer w;

	//connect QT slots and signals
	connectQt(ep, &w);

	//this step must be done after the connect in order to receive the signals for updated weights
	ep->publishAllChannels();
	ep->publishAllCategories();
	ep->publishAllWeight();

	w.show();

	return a.exec();
}
