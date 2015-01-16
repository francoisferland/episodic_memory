/*
 * networkvisualizer.h
 *
 *  Created on: 2012-10-25
 *      Author: frank
 */

#ifndef NETWORKVISUALIZER_H_
#define NETWORKVISUALIZER_H_

#include <QtGui/QMainWindow>
#include <QtGui/QToolBar>
#include <QtGui/QAction>
#include <boost/thread.hpp>
#include <queue>
#include "graphWidget.h"

#define RANDOM 0
#define EPISODE_NUMBER 1
#define NB_INPUT  7

using namespace EM_CORE;

/**
 * This class manage signal produce by the episodic memory application and adjust the view accordingly
 * It is also the main window program
 */
class NetworkVisualizer : public QMainWindow
{
	Q_OBJECT

public:
	/**
	 * Default Constructor
	 */
	NetworkVisualizer(QWidget *parent = 0);
	/**
	 * destructor
	 */
	~NetworkVisualizer();

	void joinBackgroundThread();

protected slots:
/**
 * This slot simulate a new input context in the network. There are 6 different context event randomly selected
 */
virtual void onMenuNewInput();
/**
 * This function is used when the user reset the definition of the input and channel
 * All the weights, node and channel are destroyed for every layer
 */
virtual void onMenuResetNetwork();

virtual void onClearActivationButton();

virtual void onChangeLearningMode(EM_CORE::LEARNING_MODE);
virtual void onMenuToggleLearningMode();

void onUpdateEmotionIntensity(int value);
/**
 * This function manage the view when a new category is produced in the event layer
 * the node is added and the edges are connected
 * @param index: the category index in the event layer
 */
bool onNewCategory(int index, int channelId, QString description, float vigilance, float learningRate, int layerID);

bool onNewChannel(int channelId, QString description, float relevance, int layer);

bool onUpdateWeight(int id, int indexTopCategory, int indexBottomLayer, int idLayer, float value);

bool onUpdateActivation(int index, int idChannel, float value, int idLayer);

bool onAnticipatedEvent(int index, float newVigilance, float newLearningRate, bool anticipated);
/**
 * This function is called when a user clicks on the vigilance spinbox for the event(F2) and episode(F3) layer
 * The value is applied in the model
 */
virtual int onChangeVigilance();
/**
 * This function is called when a user clicks on the learning rate spinbox for the event(F2) and episode(F3) layer
 * The value is applied in the model
 */
virtual int onChangeLearningRate();

virtual int onChangeRelevance(double relevance);

void onStartCategory();
void onStartChannel();
void onStartWeight();
void onStartActivation();
void onStartAnticipatedEvent();

protected:
GraphWidgetPtr widget;

QAction * randomEvent_;
QAction * resetNetwork_;
QAction * clearActivation_;
QAction * toggleLearningMode_;

QToolBar* getActionToolbar(){return actionToolbar;}

signals:

void sendInputData(std::map<std::string,EM_CORE::InputROSmsgPtr> mapData);
void resetNetworkSignal();
void clearActivationSignal();
void toggleLearningMode(EM_CORE::LEARNING_MODE);

void changeVigilanceSignal(int categoryId, float value);
void changeLearningRateSignal( int categoryId, float value);
void changeRelevanceSignal( int channelId, float value);

void startCategorySignal();
void startChannelSignal();
void startWeightSignal();
void startActivationSignal();
void startAnticipatedEventSignal();

private:
/**
 * This function creates the toolbar and the menu buttons
 */
 void createToolbar();
 /**
  * This function creates action associated with menu buttons
  */
 void createActions();

 void processBufferedItem();

 void tryBufferCategory();
 void tryBufferActivation();
 void tryBufferAnticipatedEvent();
 void tryBufferChannel();
 void tryBufferWeight();

 QToolBar * actionToolbar;

 int indexRandomInput;

 std::queue<GuiWeightPtr> bufferWeight;
 std::queue<GuiCategoryPtr > bufferCategory;
 std::queue<GuiActivationPtr > bufferActivation;
 std::queue<GuiChannelPtr > bufferChannel;
 std::queue<GuiAnticipatedEventPtr > bufferAnticipatedEvent;

 boost::thread threadBufferedItem;
 bool backgroundThread;

 boost::mutex mutActivation;
 boost::mutex mutAnticipatedEvent;

};


#endif /* NETWORKVISUALIZER_H_ */
