#ifndef EVENTLAYER_H_
#define EVENTLAYER_H_

#include "layerART.h"
#include "channelART.h"
#include "patternRecognizerObj.h"
#include "episodeLayer.h"
#include "memoryDecay.h"

class EventLayer;
typedef boost::shared_ptr<EventLayer> EventLayerPtr;

/**
 * This class represent the middle layer of the 3 layer ART network where the event are stored
 */
class EventLayer: public LayerART
{

public:
	/**
	 * Constructor
	 */
	EventLayer();
	/**
	 * Destructor
	 */
	virtual ~EventLayer();

	void initializeCategories();

	/**
	 * This function try to reach a resonant state with the activated category and the upper layer
	 * @return the resonant category if there is one
	 */
	CategoryARTptr resonance();

	bool createWeightPattern(CategoryARTptr & upperLayerCategory, bool fromUpperCategory);

	bool updatePatternRecognizerDatabase(PatternRecognizerObjPtr patternRecognizerObj);

	/**
	 * Display all the info in the layer (activated category)
	 */
	void printLayerInfo();

	bool clearPatternRecognizers();

	CategoryARTptr addCategory();

	bool decreaseActivation(CategoryARTptr category);

	MemoryDecayPtr getMemoryDecay() const {
		return memoryDecay;
	}

	void setMemoryDecay(const MemoryDecayPtr memoryDecay) {
		this->memoryDecay = memoryDecay;
	}

	//this function retrieves a list of inputObjPtr connected (weight > 0) to this event (excluding input complements)
	std::vector<CategoryARTptr> getInputPattern(CategoryARTptr event);

	int generateIndexCategory();

	void resetVigilance();

private :
	PatternRecognizerObjPtr m_lastActivatedCategory;
	MemoryDecayPtr memoryDecay;

};

#endif /* EVENTLAYER_H_ */
