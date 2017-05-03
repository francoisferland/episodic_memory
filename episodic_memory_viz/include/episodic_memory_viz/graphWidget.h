#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include "guiMsgObject.h"

#include <QtGui/QGraphicsView>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QProgressBar>
#include <QtGui/QLabel>
#include <iostream>

#include <boost/thread.hpp>

#include <episodic_memory_core/episodicMemoryCore.h>

#define NEURON_WIDTH 35

namespace EM_GUI
{
enum LAYER_ID {INPUT_LAYER = 1, EVENT_LAYER = 2, EPISODE_LAYER= 3, UNDEFINE = 4};
}

class Node;
typedef boost::shared_ptr<Node> NodePtr;
class Edge;
typedef boost::shared_ptr<Edge> EdgePtr;

class GraphWidget;
typedef boost::shared_ptr<GraphWidget> GraphWidgetPtr;

/**
 * This class is in charge of the disposition of the elements in the view
 */
class GraphWidget : public QGraphicsView
{
	Q_OBJECT

public:
	/**
	 * Constructor
	 */
	GraphWidget(QWidget *parent = 0);
	/**
	 * Destructor
	 */
	virtual ~GraphWidget();
	/**
	 * Paint the layers of the ART network
	 */
	void renderLayer(QPainter*, int width);
	/**
	 * instantiate spinBox for the vigilance and learning rate parameters between event and episode layer
	 */
	void addNewVigilanceSpinBox(int categoryId);
	void addNewLearningRateSpinBox(int categoryId);
	void addNewRelevanceSpinBox(int channelId);

	bool addCategory(GuiCategoryPtr category);

	bool addChannel(GuiChannelPtr channel);

	bool updateEdge(GuiWeightPtr weight);

	EdgePtr isNodesConnected(NodePtr upperNode, NodePtr lowerNode);

	/**
	 * This function update the activation value associated to a node
	 * @param weightValue : vector that contains the activation value of a layer
	 * @param layer: the layer to update
	 */
	bool updateActivationValue(int index, float activationValue, int layer);
	bool updateAnticipatedEvent(int index, float newVigilance, float newLearningRate, bool anticipated);
	/**
	 * Clears all the highlight node in a layer
	 * @param layer: the layer to clear the highlight node
	 */
	void clearHighlightNode(int layer);
	/**
	 * Clear all the visual item from the event and episode layer. clears all edges
	 */
	void clear();

	void clearAllActivation();

	std::map<int,GuiCategoryPtr> getListCategoryByChannel(int channelId);

	QDoubleSpinBox* getVigilanceSpinbox(int categoryId);
	QDoubleSpinBox* getLearningRateSpinbox(int categoryId);
	QDoubleSpinBox* getRelevanceSpinbox(int channelId);

	void setLearningRate(int categoryId, float value);
	void setVigilance(int categoryId, float value);
	void setRelevance(int channelId, float value);

	void setProgressBarValue(int value);

	bool tryLockCategory() {
		return mutCategoryRender.try_lock();
	}

	void unlockCategory()
	{
		mutCategoryRender.unlock();
	}

	bool tryLockChannel() {
		return mutChannelRender.try_lock();
	}

	void unlockChannel()
	{
		mutChannelRender.unlock();
	}

	bool tryLockWeight() {
		return mutWeightRender.try_lock();
	}

	void unlockWeight()
	{
		mutWeightRender.unlock();
	}

	bool trylockWidget()
	{
		return mutDrawing.try_lock();
	}
	void unlockWidget()
	{
		mutDrawing.unlock();
	}

	void changeLearning(EM_CORE::LEARNING_MODE mode) {
		if(mode == EM_CORE::LEARNING)
			learningMode.setText("Learning");
		else
			learningMode.setText("Recognizing");
	}

	EM_CORE::LEARNING_MODE getDisplayedLearningMode()
	{
		EM_CORE::LEARNING_MODE mode;
		if(learningMode.text().compare("Learning") == 0)
		{
			mode = EM_CORE::LEARNING;
		}
		else if(learningMode.text().compare("Recognizing") == 0)
		{
			mode = EM_CORE::RECOGNIZING;
		}
    else
    {
      mode = EM_CORE::UNDEFINED;
    }

		return mode;
	}

protected:
	/**
	 * Function that manage key press event
	 */
	void keyPressEvent(QKeyEvent *event);
	/**
	 * Function that manages wheel event
	 */
	void wheelEvent(QWheelEvent *event);
	/**
	 * function that paint the background
	 * the layers are paint here with all the nodes
	 */
	virtual void drawBackground(QPainter *painter, const QRectF &rect);
	/**
	 * function that paint the foreground
	 * the tablewidget for the recovered episode is paint here
	 */
	//virtual void drawForeground(QPainter *painter, const QRectF &rect);
	/**
	 * scale the view when zoom in or out
	 */
	void scaleView( qreal scaleFactor);

	/**
	 * Highlight the winning node with red color
	 * @param index :  index of the node
	 * @param map : the map that contains Nodes to highlight for
	 */
	void highlightWinningNode(int index, std::map<int,NodePtr> map);
	/**
	 * Clears all edges from the view
	 */
	void clearAllEdge();
	/**
	 * clears all the node in a layer from the scene
	 * @param layer: the layer where the nodes are cleared
	 */
	virtual void clearLayerNode(std::map<int, NodePtr>* layer);

	void renderChannel(QPainter * painter, int offset, int width, GuiChannelPtr channel);
	/**
	 * This function set the position of the node in the view
	 * @param idNumber : index of the node
	 * @param layer : layer that contains the node
	 */
	void renderNode(int posX, int posY, int idNumber, int layer);

	QRectF *episodeLayer;
	QRectF *eventLayer;
	QRectF *inputLayer;

	std::map<int,QDoubleSpinBox*> mapVigilanceSpinBox;
	std::map<int,QDoubleSpinBox*> mapLearningRateSpinBox;
	std::map<int,QDoubleSpinBox*> mapRelevanceSpinBox;

	std::map<int, NodePtr> listNodeInputLayer;
	std::map<int, NodePtr> listNodeEventLayer;
	std::map<int, NodePtr> listNodeEpisodeLayer;

public slots:

void zoomIn();
void zoomOut();

private :

std::map<int, GuiWeightPtr> mapGuiWeight;
std::map<int, GuiCategoryPtr> mapGuiCategory;
std::map<int, GuiChannelPtr> mapGuiChannel;

std::map<int,QGraphicsProxyWidget*> mapProxyWidgetVigilance;
std::map<int,QGraphicsProxyWidget*> mapProxyWidgetLearningRate;
std::map<int,QGraphicsProxyWidget*> mapProxyWidgetRelevance;

boost::mutex mutWeightRender;
boost::mutex mutCategoryRender;
boost::mutex mutChannelRender;
boost::timed_mutex mutDrawing;

QLabel learningMode;
QProgressBar emotionProgress_;

int totalWidth_;

};

#endif
