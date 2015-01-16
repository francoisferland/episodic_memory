#ifndef INPUTLAYER_H_
#define INPUTLAYER_H_

#include <boost/numeric/ublas/vector.hpp>
#include <utilite/UtiLite.h>
#include <utilite/UTimer.h>
#include "channelART.h"
#include "inputObj.h"
#include "layerART.h"
#include "eventLayer.h"

using namespace boost::numeric::ublas;

class InputLayer;
typedef boost::shared_ptr<InputLayer> InputLayerPtr;

/**
 * This class represent the botom layer of the 3 layer ART network where the inputs are processed
 */
class InputLayer : public LayerART {
public:
	/**
	 * Constructor
	 */
	InputLayer();
	/**
	 * Desctructor
	 */
	virtual ~InputLayer();

	void initializeCategories();

	/**
	 * This function try to reach a resonant state with the activated inputs and the upper layer
	 * @return false if no category matches the input, true: if a category is close enough
	 */
	CategoryARTptr resonance();
	/**
	 * This function adds a new category input for this layer
	 * @param description : a descriptive string of the input
	 * @param channelName : the corresponding channel
	 */
	InputObjPtr addNewInput(std::string description, std::string channelDescription);

	bool activateInputs(std::map<std::string,InputROSmsgPtr> mapActiveInput);

	/**
	 * This function update the input Table in the database according to current data in the network
	 * @param inputPtr : shared pointer to a input mapped object
	 */
	bool updateInputDatabase(InputObjPtr);

	/**
	 * display information related to the input layer
	 */
	void printLayerInfo();

	InputObjPtr getCategoryByDescription(std::string description);

	bool clearInputs();

	int generateIndexCategory();


};

#endif /* INPUTLAYER_H_ */
