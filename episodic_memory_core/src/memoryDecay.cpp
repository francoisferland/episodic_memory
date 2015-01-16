#include "memoryDecay.h"

MemoryDecay::MemoryDecay()
{
	alphaParameter = 0;
}

MemoryDecay::~MemoryDecay()
{
}

/*double MemoryDecay::decreaseActivation(double activationValue)
{
	//activation value is not used, only depends on time

	//Decay function
	// r(t) = alpha * e^(-alpha * t)
	// v* r(t) / (b + integral(r(t)))

	int v = 10;
	int b = 3;
	//convert TimeDuration in second
	int secondDuration = m_timeDuration.seconds();
	double rt =  std::exp(-alphaParameter * secondDuration);
	double value = v * alphaParameter * rt / (3 - rt);

	if(value < 0.0001)
		value = 0;

	return value;
}
*/


double MemoryDecay::decreaseActivation(double activationValue)
{
	//by default the activation value is decrease linearly
	activationValue = activationValue * (1 - DECAY_FACTOR);

	if(activationValue <= 0)
		activationValue = 0;

	return activationValue;
}
