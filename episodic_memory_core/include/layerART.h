#ifndef LAYERART_H_
#define LAYERART_H_

#include <QtCore/QObject>
#include "channelART.h"
#include "inputDao.h"
#include "channelDao.h"
#include "weightDao.h"
#include "patternRecognizerDao.h"
#include "ros/ros.h"

using namespace EM_CORE;

namespace EM_CORE
{
enum LAYER_ID {INPUT_LAYER = 1, EVENT_LAYER = 2, EPISODE_LAYER= 3, UNDEFINE = 4};
enum LEARNING_MODE {LEARNING,RECOGNIZING};
}

class LayerART;
typedef boost::shared_ptr<LayerART> LayerARTptr;

/**
 * Generic class to define a layer in the ART network architecture. 3 layer are used in the episodic memory
 */
class LayerART : public QObject
{
	Q_OBJECT

public:

	LayerART();
	virtual ~LayerART(){};

	virtual void initializeCategories(){}
	/**
	 * this function adds a new channel for the layer
	 * @param p_description : a descriptive string of the channel
	 * @param p_vigilance : the vigilance parameter for the channel
	 * @param p_learningRate : the learning rate parameter for the channel
	 */
	virtual ChannelARTptr addChannel(std::string p_description, float p_relevance);
	virtual CategoryARTptr addCategory(){return CategoryARTptr();}
	virtual WeightObjPtr addWeight(CategoryARTptr downCategory,CategoryARTptr upCategory, float weightValue);
	virtual int generateIndexCategory(){return -1;}

	bool fillWeightMatrix(matrix<double> & weightMatrix, std::map<int,CategoryARTptr> mapDownCategory);

	virtual bool learnPattern(CategoryARTptr & category);
	virtual bool createWeightPattern(CategoryARTptr & upperLayerCategory, bool fromUpperCategory);
	virtual bool decreaseActivation(CategoryARTptr category);

	bool updateChannelDatabase(ChannelARTptr channel);
	bool updateWeightDatabase(WeightObjPtr weightObj);

	CategoryARTptr recognition();

	bool clearChannels();
	bool clearWeights();
	bool clearActivation();

	void setUpperLayer(LayerARTptr layer){ this->m_upperLayer = layer; }
	void setLowerLayer(LayerARTptr layer){ this->m_lowerLayer = layer; }
	const LayerARTptr& getLowerLayer() const {	return m_lowerLayer; }
	const LayerARTptr& getUpperLayer() const {	return m_upperLayer; }

	bool isUpperLayer();
	bool isLowerLayer();

	void setCurrentEmotionIntensity(float emotionIntensity){currentEmotionIntensity_ = emotionIntensity;}

	LEARNING_MODE getLearningMode(){return m_learningMode;}
	void setLearningMode(LEARNING_MODE learningMode){m_learningMode = learningMode;}

	void setLayerID(LAYER_ID layerId){m_layerId = layerId;}
	LAYER_ID getLayerId(){return m_layerId;}

	void setDao(WeightDaoPtr dao){m_weightDao = dao;}
	void setDao(InputDaoPtr dao){m_inputDao = dao;}
	void setDao(ChannelDaoPtr dao){m_channelDao = dao;}
	void setDao(PatternRecognizerDaoPtr dao){m_patternRecognizerDao = dao;}

	void setListChannel(std::map<int,ChannelARTptr> listChannel){m_mapChannel = listChannel;}
	void setListWeight(std::map<int,WeightObjPtr> listWeight){m_mapWeight = listWeight;}

	std::map<int,ChannelARTptr> getMapChannel(){return m_mapChannel;}

	std::map<int,CategoryARTptr> getListCategory();
	std::vector<float> getVigilanceVect();
	ChannelARTptr getChannelByDescription(std::string description);
	CategoryARTptr getCategoryByIndex(int index);

	CategoryARTptr getLastActivatedCategory();
	CategoryARTptr getLatestActivatedCategory(CategoryARTptr upperCategory);

	void notifyNewChannel(ChannelARTptr channel);
	void notifyNewCategory(InputObjPtr input);
	void notifyNewCategory(PatternRecognizerObjPtr input);
	void notifyActivation(bool updatedOnly);
	void notifyUpdatedWeight(CategoryARTptr upperCategory);
	void notifyAnticipatedEvent(PatternRecognizerObjPtr pattern);

	void notifyPatternRecognizerParameter();
	void notifyPatternRecognizerParameter(CategoryARTptr);
	void notifyAllWeight();
	void notifyAllCategories();
	void notifyAllChannels();

protected:

	LayerARTptr m_upperLayer;
	LayerARTptr m_lowerLayer;

	LAYER_ID m_layerId;
	LEARNING_MODE m_learningMode;

	std::map<int,ChannelARTptr> m_mapChannel;
	std::map<int,WeightObjPtr> m_mapWeight;

	WeightDaoPtr m_weightDao;
	InputDaoPtr m_inputDao;
	ChannelDaoPtr m_channelDao;
	PatternRecognizerDaoPtr m_patternRecognizerDao;

	float currentEmotionIntensity_;

	signals:
	/**
	 * signal emitted when a new category is created
	 */
	void newCategorySignal(int idCategory,int channelUid, QString description, float vigilance, float learningRate, int layerId);
	/**
	 * signal emitted when a category is updated
	 */
	void updateWeightSignal(int id, int idTopNode, int idBottomNode, int topLayerId, float weightValue);
	/**
	 * signal emitted when a new channel is created
	 */
	void newChannelSignal(int channelUid,QString description,float relevance, int layerId);

	void updateActivationSignal(int uidCategory, int idChannel,float activationValue, int layerId);

	void anticipatedEventSignal(int index, float newVigilance, float newLearningRate, bool isAnticipated);

};

#endif
