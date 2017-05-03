#ifndef CATEGORYART_H_
#define CATEGORYART_H_

#include <boost/thread.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"
//#include <utilite/UtiLite.h>

using namespace boost::posix_time;

#define DEFAULT_VIGILANCE 0.55
#define DEFAULT_LEARNING_RATE 0.6
#define MIN_LEARNIN_RATE 0.1
#define MAX_LEARNING_RATE 1
#define MIN_VIGILANCE 0.45
#define MAX_VIGILANCE 0.85

/**
 * This class represent a neuron in a layer in a ART network
 */

class ChannelART;
class WeightObj;

class CategoryART;
typedef boost::shared_ptr<CategoryART> CategoryARTptr;


class CategoryART {
public:
	/**
	 * Constructor
	 */
	CategoryART();
	CategoryART(int uid);

	/**
	 * Desctructor
	 */
	virtual ~CategoryART();
	/**
	 * Set the id number for this category
	 */
	void setUID(int uid){ m_uid = uid; }
	/**
	 * Set the activation Value for this category
	 */
	void setActivationValue(double p_activationValue);
	/**
	 * return the activation value
	 */
	double getActivationValue();

	void clearOnChange();
	/**
	 * return the id number
	 */
	int getUid(){ return m_uid; }
	/**
	 * print in console informations relative to this category
	 */
	virtual void print();
	/**
	 * memorizes the last moment this neuron was activated
	 */
	void setActivatedTime();
	/**
	 * compute the time elapsed since the last activation
	 */
	time_duration getElapseTime();

	bool isActivationChange() const {
		return m_isActivationChange;
	}

	int getIndexCategory() const {
		return m_indexCategory;
	}

	void setIndexCategory(int indexCategory) {
		this->m_indexCategory = indexCategory;
	}

	boost::shared_ptr<ChannelART> getChannel() const;

	void setChannel(boost::shared_ptr<ChannelART> channelPtr);

	void addUpWeight(boost::shared_ptr<WeightObj> weight);
	void addDownWeight(boost::shared_ptr<WeightObj> weight);

	std::map<int,boost::shared_ptr<WeightObj> > getMapUpWeight(){return m_mapUpWeight;}
	std::map<int,boost::shared_ptr<WeightObj> > getMapDownWeight(){return m_mapDownWeight;}

	boost::shared_ptr<WeightObj> findWeightByConnection(CategoryARTptr category);
	float getMinimumVigilance(int & nbActiveInputs);

private:

	int m_uid;
	int m_indexCategory;
	double m_activationValue;
	bool m_isActivationChange;
	ptime m_lastTimeActivated;
	boost::shared_ptr<ChannelART> m_channelPtr;

	std::map<int,boost::shared_ptr<WeightObj> > m_mapUpWeight;
	std::map<int,boost::shared_ptr<WeightObj> > m_mapDownWeight;
};

#endif
