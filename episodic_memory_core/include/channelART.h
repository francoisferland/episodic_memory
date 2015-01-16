#ifndef CHANNELART_H_
#define CHANNELART_H_

#include "channelObj.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/algorithm/minmax.hpp>
#include "ros/ros.h"

using namespace boost::numeric::ublas;

#define DEFAULT_EPISODE_SALIENCY 1
#define DEFAULT_EVENT_SALIENCY 0.8

class ChannelART;
typedef boost::shared_ptr<ChannelART> ChannelARTptr;
class CategoryART;

struct ResonanceInfo
{
	ResonanceInfo():relativeIndexCategory(-1),matchValue(0),isResonant(false){}
	ResonanceInfo(int indexCategory,float matchValue, bool isResonant):
		relativeIndexCategory(indexCategory),matchValue(matchValue),isResonant(isResonant){}
	int relativeIndexCategory;
	float matchValue;
	bool isResonant;
};

/**
 * This class is used to represent a channel in a layer ART. Channel are used to regroup similar semantic inputs.
 * Each channel can have his own vigilance and learning rate parameter
 */

class ChannelART {
public:
	/**
	 * Constructor
	 */
	ChannelART();
	/**
	 * Destructor
	 */
	virtual ~ChannelART(){}

	/**
	 * Update the channel and the weightMatrix when a new input is added
	 * @param neuron: the category (or neuron) to add to this channel
	 * @param updateDatabase: true if it is the first time this input is added, false if it comes from the database
	 * @return the id number of the added neuron
	 */
	int addInput(boost::shared_ptr<CategoryART> category);
	/**
	 * Activates an input neuron
	 * @param p_index : index of the neurons
	 * @param p_activationValue : activation value between 1 and 0
	 * @return true if neuron is activated successfully
	 */
	bool activateInput(int p_index, float p_activationValue);
	/**
	 * The main function of the ART network uses to process the input through the network
	 * @return map containing the category index as the key, and the resonant state (bool) paired with the match value (double) as the value
	 */
	std::map<int,ResonanceInfo> resonance(matrix<float> weightMatrix,std::vector<float> vigilanceVect);

	float modulateVigilanceWithRelevance(float vigilance);

	std::map<int,float> activateCategory(matrix<float> & weightMatrix, vector<float> p_input);

	double calculateMatch(matrix<float> & weightMatrix, int categoryIndex, vector<float> p_input);

	const ChannelObjPtr getChannelObj() const {
		return m_channelObj;
	}

	void setChannelObj(const ChannelObjPtr channelObj) {
		m_channelObj = channelObj;
	}

	/**
	 * Returns a pointer to the list of the channel's category/neurons
	 */
	std::map<int,boost::shared_ptr<CategoryART> > getMapCategories();
	void setMapCategory( std::map<int, boost::shared_ptr<CategoryART> > mapCategory) {
		m_mapCategory = mapCategory;
	}

	/**
	 * Returns the id number
	 */
	int getIndexChannel() const { return m_channelObj->getIndexChannel(); }
	int getUid() const { return m_channelObj->getUid();}
	/**
	 * Set the id number for this channel
	 * Each channel is identified with a number
	 */
	void setIndexChannel(int p_idChannel){ m_channelObj->setIndexChannel(p_idChannel); }
	/**
	 * Returns the description
	 */
	std::string getDescription() const { return m_channelObj->getDescription(); }
	/**
	 * Set the description
	 * each channel uses a textual identifier
	 */
	void setDescription(std::string p_description){ m_channelObj->setDescription(p_description); }
	/**
	 * Returns the vigilance parameter
	 */
	float getRelevance() const {return m_channelObj->getRelevance();}
	/**
	 * Set the vigilance parameter for this channel
	 * each channel may uses a different vigilance parameter, so
	 * some channel may be less important for a spatio-temporal context
	 */
	void setRelevance(float relevance){ m_channelObj->setRelevance(relevance);}

	void setLayerId(int layerId){m_channelObj->setLayerId(layerId);}

	/**
	 * Returns the number of category (or neurons) associated with this channel
	 */
	int getNbCategory();

	/**
	 * Getter function, Activation value vect associated with this channel
	 * @return vector of activation values
	 */
	vector<float> getActivationValueVect();

	/**
	 * prints all the categories with their activation value
	 */
	void printCategories();

	void setDebugResonance(bool debugResonance) {
		ROS_WARN_COND(debugResonance,"debug resonance ");
		this->debugResonance = debugResonance;
	}

private:

	ChannelObjPtr m_channelObj;
	std::map<int,boost::shared_ptr<CategoryART> > m_mapCategory;

	bool debugResonance;

};

#endif /* CHANNELART_H_ */
