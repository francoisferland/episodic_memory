#ifndef EPISODICMEMORYCORE_H_
#define EPISODICMEMORYCORE_H_

#include <sqlite3.h>
//#include <utilite/UtiLite.h>
#include <boost/numeric/ublas/vector.hpp>
#include "episodeLayer.h"
#include "eventLayer.h"
#include "inputLayer.h"
#include "channelART.h"
#include "daoFactory.h"
#include "weightObj.h"
#include "memoryDecay.h"
#include <QtCore/QObject>

using namespace boost::numeric::ublas;

namespace EM_CORE
{
class EpisodicMemoryCore;
typedef boost::shared_ptr<EpisodicMemoryCore> EpisodicMemoryCorePtr;

/**
 * Main class for the episodic memory
 * This version is a specialized class for the ART network. if another model is used, this class must inherit another class
 */
class EpisodicMemoryCore : public QObject {

	Q_OBJECT

public:
	/**
	 * Constructor
	 * @param databasePath : path to sqlite database
	 */
	EpisodicMemoryCore(const char** databasePath);
	virtual ~EpisodicMemoryCore(){}
	/**
	 * this function initialize member of the class when the constructor is called
	 * @return true if initialization succeed
	 */
	bool initialize();

	CategoryARTptr processInput();

	/**
	 * This function generate a vector coherent with the network description
	 * @param map that contains all the active inputs desctiption along with their respective value
	 * @return a vector that contains all the active inputs from the arguments and the inactive one
	 */
	bool activateInputs(std::map<std::string,InputROSmsgPtr> inputROSmsgMap);

	void publishAllWeight();
	void publishAllCategories();
	void publishAllChannels();
	void publishAllActivation();

	/**
	 * Configuration of the network
	 * Add a fusion-ART channel for classification of the inputs
	 * @param description : the name of the channel
	 * @param relevance : parameter used for comparison matching
	 */
	ChannelARTptr addInputChannel(std::string description, float relevance);

	/**
	 * Configuration of the network
	 * Add a input fields associated with a channel
	 * @param inputName : name of the input (used for visualisation and debug purpose)
	 * @param channelName : name of the associated channel
	 */
	InputObjPtr addInput(std::string inputName, std::string channelName);

	void adjustVigilance(std::vector<CategoryARTptr> anticipatedEvents);
	//void adjustLearningRate(LayerARTptr layer);

	/**
	 * Getter function, shared pointer to the episode layer
	 */
	EpisodeLayerPtr getEpisodeLayer(){return this->m_episodeLayer;}
	/**
	 * Getter function, shared pointer to the event layer
	 */
	EventLayerPtr getEventLayer(){return this->m_eventLayer;}
	/**
	 * Getter function, shared pointer to the input layer
	 */
	InputLayerPtr getInputLayer(){return this->m_inputLayer;}
	/**
	 * Getter function, shared pointer to decay agent
	 */
	MemoryDecayPtr getDecay(){return m_decay;}
	void setDecayAgent(MemoryDecayPtr  p_decay){ m_decay = p_decay;}

	/**
	 * Getter function, database connection object
	 */
	sqlite3 *getDbPtr(){return db;}

	/**
	 * call the parent function to print network information
	 */
	void printNetwork();

	void clearChannels();
	void clearWeights();
	void clearInputs();

public slots:

void onReceiveData(std::map<std::string,EM_CORE::InputROSmsgPtr> mapData);
void onEmotionChange(float emotionIntensity);
std::vector<std::vector<CategoryARTptr> > recallLastActivatedEpisode();

void setVigilance(int indexCategory, float value);
void setLearningRate(int indexCategory, float value);
void setRelevance(int indexChannel, float value);
void setLearningRateMode(EM_CORE::LEARNING_MODE mode);

void clearActivation();
void clearEpisodicMemory();

private:

EpisodeLayerPtr m_episodeLayer;
EventLayerPtr m_eventLayer;
InputLayerPtr m_inputLayer;
MemoryDecayPtr m_decay;

float emotionIntensity_;

const char** m_databasePath;
sqlite3 *db;

};

}

#endif
