#ifndef EPISODELAYER_H_
#define EPISODELAYER_H_

#include "layerART.h"
#include "eventLayer.h"
#include "categoryART.h"

class EpisodeLayer;
typedef boost::shared_ptr<EpisodeLayer> EpisodeLayerPtr;


/**
 * This class represent the upper layer of the 3 layer ART network where the episode are stored
 */
class EpisodeLayer: public LayerART {

	Q_OBJECT

public:
	/**
	 * Constructor
	 */
	EpisodeLayer();
	/**
	 * Destructor
	 */
	virtual ~EpisodeLayer();

	CategoryARTptr addCategory();

	int generateIndexCategory();

	bool learnPattern(CategoryARTptr & category);

	void computeVigilance(CategoryARTptr & category);
	/**
	 * Function used to build a recovered episode through the event and input layer
	 * a signal is emitted for any subscriber to receive the vector that contains the data
	 * @param winnerNode: index of the episode recovered
	 *
	 */
	std::vector<CategoryARTptr> buildRecognizedEpisode(CategoryARTptr winnerNode);

	void initializeCategories();

	bool updatePatternRecognizerDatabase(PatternRecognizerObjPtr patternRecognizerObj);

	std::vector<CategoryARTptr> recallLastActivatedEpisode();

	/**
	 * Display all the info in the layer (activated category)
	 */
	void printLayerInfo();

	bool clearPatternRecognizers();

	void displayBuiltEpisode(std::vector<CategoryARTptr> episode);

	void notifyLearningModeChanged();

signals:
	/**
	 * Signal emitted when an episode is recognised and build as a list of events
	 */
	void episodeRecognised(	std::vector<CategoryARTptr> );

	void newEpisodeLearned(int categoryId);

	void learningModeChanged(EM_CORE::LEARNING_MODE);

};



#endif /* EPISODELAYER_H_ */
