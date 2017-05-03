#ifndef SHORTMEMORYTIMEDECAY_H_
#define SHORTMEMORYTIMEDECAY_H_

#include <boost/date_time/posix_time/posix_time.hpp>
#include <math.h>

using namespace boost::posix_time;

class MemoryDecay;
typedef boost::shared_ptr<MemoryDecay> MemoryDecayPtr;

#define DECAY_FACTOR 0.05

/**
 * This class is a decay agent for the event layer.
 * The decaying is based on a exponential function that depends on time
 */
class MemoryDecay {
public:
	/**
	 * Constructor
	 */
	MemoryDecay();
	/**
	 * Destructor
	 */
	virtual ~MemoryDecay();
	/**
	 * This function decreases the activation value based on Time
	 * @param activationValue: the current activation value of the event, NOT USED
	 * @return: the value computed
	 */
	double decreaseActivation(double activationValue);
	/**
	 *	Set the duration from the last time the event was activated. This param is used the the decreaseActivation function
	 *	@param duration: time duration object
	 */
	void setDuration(time_duration duration){m_timeDuration = duration;}
	/**
	 * Set the alpha parameter of the decay function. It influences the shape of the curve
	 * @param alpha: between 0 and 1
	 */
	void setAlphaParameter(double alpha){alphaParameter = alpha;}

private :

	time_duration m_timeDuration;
	double alphaParameter; //emotion salience

};


#endif /* SHORTMEMORYTIMEDECAY_H_ */
