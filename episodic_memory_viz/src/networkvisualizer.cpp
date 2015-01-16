/*
 * networkvisualizer.cpp
 *
 *  Created on: 2012-10-25
 *      Author: frank
 */
#include "networkvisualizer.h"

NetworkVisualizer::NetworkVisualizer( QWidget *parent)
{
	setGeometry(15,25,1280,780);

	actionToolbar = NULL;
	randomEvent_ = NULL;
	resetNetwork_ = NULL;
	clearActivation_ = NULL;
	toggleLearningMode_ = NULL;

	indexRandomInput = -1;

	widget.reset();
	widget = GraphWidgetPtr(new GraphWidget);

	//start a thread that try to process all the buffered items
	backgroundThread = true;

	threadBufferedItem = boost::thread(&NetworkVisualizer::processBufferedItem,this);

	QObject::connect(this,SIGNAL(startCategorySignal()),
			this, SLOT(onStartCategory()));
	QObject::connect(this,SIGNAL(startChannelSignal()),
			this, SLOT(onStartChannel()));
	QObject::connect(this,SIGNAL(startWeightSignal()),
			this, SLOT(onStartWeight()));
	QObject::connect(this,SIGNAL(startActivationSignal()),
			this, SLOT(onStartActivation()));
	QObject::connect(this,SIGNAL(startAnticipatedEventSignal()),
			this, SLOT(onStartAnticipatedEvent()));

	createActions();
	createToolbar();

	this->setCentralWidget(widget.get());
}

NetworkVisualizer::~NetworkVisualizer()
{
	joinBackgroundThread();
}

void NetworkVisualizer::joinBackgroundThread()
{
	backgroundThread = false;
	threadBufferedItem.join();
}

void NetworkVisualizer::processBufferedItem()
{
	UINFO("Start thread");
	while(backgroundThread)
	{
		tryBufferChannel();
		tryBufferCategory();
		tryBufferActivation();
		tryBufferWeight();
		tryBufferAnticipatedEvent();
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}
	UINFO("end thread");
}

void NetworkVisualizer::onMenuNewInput()
{
	if(RANDOM)
	{
		indexRandomInput = qrand() % (NB_INPUT)  ;
	}
	else
	{
		indexRandomInput++;
		if(indexRandomInput >= NB_INPUT)
			indexRandomInput = 0;
	}

	std::map<std::string,InputROSmsgPtr> testMap;
	std::map<std::string,InputROSmsgPtr> testMap2;
	std::map<std::string,InputROSmsgPtr> testMap3;
	std::map<std::string,InputROSmsgPtr> testMap4;
	std::map<std::string,InputROSmsgPtr> testMap5;
	std::map<std::string,InputROSmsgPtr> testMap6;
	std::map<std::string,InputROSmsgPtr> testMap7;

#if RANDOM

	testMap["SpongeBob"] = InputROSmsgPtr(new InputROSmsg("SpongeBob","Persons",1,10));
	testMap["Orange ball"] = InputROSmsgPtr(new InputROSmsg("Orange ball","Object",1,10));
	testMap["Labo"] = InputROSmsgPtr(new InputROSmsg("Labo","Location",1,10));
	testMap["Surprise"] = InputROSmsgPtr(new InputROSmsg("Surprise","Emotion",0.7,10));

	testMap2["Chuck Norris"] = InputROSmsgPtr(new InputROSmsg("Chuck Norris","Persons",1,10));
	testMap2["Labo"] = InputROSmsgPtr(new InputROSmsg("Labo","Location",1,10));
	testMap2["Joy"] = InputROSmsgPtr(new InputROSmsg("Joy","Emotion",0.3,10));

	testMap3["Chuck Norris"] = InputROSmsgPtr(new InputROSmsg("Chuck Norris","Persons",1,10));
	testMap3["Labo"] = InputROSmsgPtr(new InputROSmsg("Labo","Location",1,10));
	testMap3["Follow"] = InputROSmsgPtr(new InputROSmsg("Follow","Motivation",1,10));

	testMap4["Chuck Norris"] = InputROSmsgPtr(new InputROSmsg("Chuck Norris","Persons",1,10));
	testMap4["Labo"] = InputROSmsgPtr(new InputROSmsg("Labo","Location",1,10));
	testMap4["Red ball"] = InputROSmsgPtr(new InputROSmsg("Red ball","Object",1,10));
	testMap4["Follow"] = InputROSmsgPtr(new InputROSmsg("Follow","Motivation",1,10));

	testMap5["SpongeBob"] = InputROSmsgPtr(new InputROSmsg("SpongeBob","Persons",1,10));
	testMap5["Labo"] = InputROSmsgPtr(new InputROSmsg("Labo","Location",1,10));
	testMap5["Red ball"] = InputROSmsgPtr(new InputROSmsg("Red ball","Object",1,10));
	testMap5["Follow"] = InputROSmsgPtr(new InputROSmsg("Follow","Motivation",1,10));

	testMap6["SpongeBob"] = InputROSmsgPtr(new InputROSmsg("SpongeBob","Persons",1,10));
	testMap6["Cafeteria"] = InputROSmsgPtr(new InputROSmsg("Cafeteria","Location",1,10));
	testMap6["Orange ball"] = InputROSmsgPtr(new InputROSmsg("Orange ball","Object",1,10));
	testMap6["Blue ball"] = InputROSmsgPtr(new InputROSmsg("Blue ball","Object",1,10));
	testMap6["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));


#else


#if EPISODE_NUMBER == 1

	testMap["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap2["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	testMap2["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));
	testMap2["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.3,10));
	testMap2["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));

	testMap3["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap3["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap3["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));

	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap4["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap4["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap4["Prendre"] = InputROSmsgPtr(new InputROSmsg("Prendre","Motivation",1,10));

	testMap5["Colere"] = InputROSmsgPtr(new InputROSmsg("Colere","Emotion",1,10));
	testMap5["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap5["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap5["Prendre"] = InputROSmsgPtr(new InputROSmsg("Prendre","Motivation",1,10));

	testMap6["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap6["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap6["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap6["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));

	testMap7["Personne A"] = InputROSmsgPtr(new InputROSmsg("Personne A","Persons",1,10));
	testMap7["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap7["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap7["Donner"] = InputROSmsgPtr(new InputROSmsg("Donner","Motivation",1,10));
	testMap7["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));

#elif EPISODE_NUMBER == 2

	testMap["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap2["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap2["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));
	testMap2["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",0.3,10));
	testMap2["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));

	testMap3["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap3["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap3["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));

	testMap4["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap4["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap4["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap4["Prendre"] = InputROSmsgPtr(new InputROSmsg("Prendre","Motivation",1,10));

	testMap5["Colere"] = InputROSmsgPtr(new InputROSmsg("Colere","Emotion",1,10));
	testMap5["Cuisine"] = InputROSmsgPtr(new InputROSmsg("Cuisine","Location",1,10));
	testMap5["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap5["Prendre"] = InputROSmsgPtr(new InputROSmsg("Prendre","Motivation",1,10));

	testMap6["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap6["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap6["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));
	testMap6["Go To"] = InputROSmsgPtr(new InputROSmsg("Go To","Motivation",1,10));

	testMap7["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap7["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap7["Sac chips"] = InputROSmsgPtr(new InputROSmsg("Sac chips","Object",1,10));
	testMap7["Donner"] = InputROSmsgPtr(new InputROSmsg("Donner","Motivation",1,10));
	testMap7["Joie"] = InputROSmsgPtr(new InputROSmsg("Joie","Emotion",1,10));

#elif EPISODE_NUMBER == 3
	testMap["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap2["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap2["Personne C"] = InputROSmsgPtr(new InputROSmsg("Personne C","Persons",1,10));
	testMap2["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap2["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap2["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));

	testMap3["Personne B"] = InputROSmsgPtr(new InputROSmsg("Personne B","Persons",1,10));
	testMap3["Attente"] = InputROSmsgPtr(new InputROSmsg("Attente","Motivation",1,10));
	testMap3["Salon"] = InputROSmsgPtr(new InputROSmsg("Salon","Location",1,10));
	testMap3["Neutre"] = InputROSmsgPtr(new InputROSmsg("Neutre","Emotion",0.7,10));
#endif

#endif

	switch(indexRandomInput)
	{
	case 0:
		emit sendInputData(testMap);
		break;
	case 1:
		emit sendInputData(testMap2);
		break;
	case 2:
		emit sendInputData(testMap3);
		break;
	case 3 :
		emit sendInputData(testMap4);
		break;
	case 4:
		emit sendInputData(testMap5);
		break;
	case 5 :
		emit sendInputData(testMap6);
		break;
	case 6 :
		emit sendInputData(testMap7);
		break;
	}

}

void NetworkVisualizer::onMenuResetNetwork()
{
	//clear view and clear database
	UINFO("On menu reset network");
	widget->clear();
	emit resetNetworkSignal();
}

void NetworkVisualizer::onClearActivationButton()
{
	emit clearActivationSignal();
	UINFO("Clear Activation");
	widget->clearAllActivation();
}

void NetworkVisualizer::onMenuToggleLearningMode()
{
	EM_CORE::LEARNING_MODE mode = widget->getDisplayedLearningMode();

	if(mode == EM_CORE::LEARNING)
		mode = EM_CORE::RECOGNIZING;
	else if (mode == EM_CORE::RECOGNIZING)
		mode = EM_CORE::LEARNING;


	emit toggleLearningMode(mode);
}

void NetworkVisualizer::onChangeLearningMode(EM_CORE::LEARNING_MODE mode)
{
	//UINFO("Gui receive learning mode %i\n",mode);
	widget->changeLearning(mode);
}

bool NetworkVisualizer::onNewChannel(int channelId, QString description,float relevance, int layer)
{
	GuiChannelPtr bufChannel = GuiChannelPtr(new GuiChannel(channelId,description,relevance,layer));

	UINFO("received new channel %i %s %5.2f %i",channelId,description.toStdString().c_str(),relevance,layer);
	while(!widget->tryLockChannel())
		boost::this_thread::sleep(boost::posix_time::milliseconds(2));
	bufferChannel.push(bufChannel);
	widget->unlockChannel();
	return true;
}

bool NetworkVisualizer::onNewCategory(int index, int channelId, QString description, float vigilance, float learningRate, int layerId)
{
	GuiCategoryPtr bufCategory = GuiCategoryPtr(new GuiCategory(index, channelId,description,vigilance,learningRate,layerId));
	UINFO("new category %i %s v=%5.2f",index,description.toStdString().c_str(),vigilance);
	while(!widget->tryLockCategory())
		boost::this_thread::sleep(boost::posix_time::milliseconds(2));
	bufferCategory.push(bufCategory);
	widget->unlockCategory();
	return true;
}

bool NetworkVisualizer::onUpdateWeight(int id, int indexTopCategory, int indexBottomLayer, int idLayer, float value)
{
	UINFO("updatedWeight %i between %i and %i",id, indexTopCategory,indexBottomLayer);
	GuiWeightPtr bufWeight = GuiWeightPtr(new GuiWeight(id, indexTopCategory,indexBottomLayer,idLayer,value));
	while(!widget->tryLockWeight())
		boost::this_thread::sleep(boost::posix_time::milliseconds(2));
	bufferWeight.push(bufWeight);
	widget->unlockWeight();
	return true;
}

bool NetworkVisualizer::onUpdateActivation(int index, int idChannel, float value, int idLayer)
{
	GuiActivationPtr bufActivation = GuiActivationPtr(new GuiActivation(index, idChannel, value, idLayer));
	while(!mutActivation.try_lock())
		boost::this_thread::sleep(boost::posix_time::milliseconds(2));
	bufferActivation.push(bufActivation);
	mutActivation.unlock();
	return true;
}

bool NetworkVisualizer::onAnticipatedEvent(int index, float newVigilance, float newLearningRate, bool anticipated)
{
	GuiAnticipatedEventPtr bufAnticipatedEvent = GuiAnticipatedEventPtr(new GuiAnticipatedEvent(index,newVigilance, newLearningRate, anticipated));
	while(!mutAnticipatedEvent.try_lock())
		boost::this_thread::sleep(boost::posix_time::milliseconds(5));
	bufferAnticipatedEvent.push(bufAnticipatedEvent);
	mutAnticipatedEvent.unlock();
	return true;
}

void NetworkVisualizer::tryBufferCategory(){

	if(bufferCategory.size() > 0)
	{
		if(widget->tryLockCategory())
		{
			emit startCategorySignal();
		}
	}
}

void NetworkVisualizer::tryBufferActivation(){

	if(bufferActivation.size() > 0)
	{
		if(mutActivation.try_lock())
		{
			emit startActivationSignal();
		}
	}
}

void NetworkVisualizer::tryBufferAnticipatedEvent(){

	if(bufferAnticipatedEvent.size() > 0)
	{
		if(mutAnticipatedEvent.try_lock())
		{
			emit startAnticipatedEventSignal();
		}
	}
}

void NetworkVisualizer::tryBufferWeight(){

	if(bufferWeight.size() > 0)
	{
		if(widget->tryLockWeight())
		{
			emit startWeightSignal();
		}
	}
}

void NetworkVisualizer::tryBufferChannel()
{
	if(bufferChannel.size() > 0)
	{
		if(widget->tryLockChannel())
		{
			emit startChannelSignal();
		}
	}
}

void NetworkVisualizer::onStartCategory()
{
	GuiCategoryPtr category = bufferCategory.front();
	//if(widget->trylockWidget())
	if( category )
	{
		if(widget->addCategory(GuiCategoryPtr(category)))
		{
			if(category->getLayerId() == EVENT_LAYER || category->getLayerId() == EPISODE_LAYER)
			{
				QDoubleSpinBox* spinBoxVigilance = widget->getVigilanceSpinbox(category->getIndex());
				QObject::connect(spinBoxVigilance, SIGNAL(editingFinished()), this, SLOT(onChangeVigilance()),Qt::UniqueConnection);

				QDoubleSpinBox* spinBoxLearningRate = widget->getLearningRateSpinbox(category->getIndex());
				QObject::connect(spinBoxLearningRate, SIGNAL(editingFinished()), this, SLOT(onChangeLearningRate()),Qt::UniqueConnection);

			}
			bufferCategory.pop();
		}

	}
	//widget->unlockWidget();
	widget->unlockCategory();
	//UINFO("end try category, buffer size %i",bufferCategory.size());

}

void NetworkVisualizer::onStartChannel()
{
	GuiChannelPtr channel = bufferChannel.front();

	//if(widget->trylockWidget())
	if(channel)
	{
		if(widget->addChannel(GuiChannelPtr(channel)))
		{
			QDoubleSpinBox* spinBox = widget->getRelevanceSpinbox(channel->getChannelId());
			QObject::connect(spinBox, SIGNAL(valueChanged(double)), this, SLOT(onChangeRelevance(double)),Qt::UniqueConnection);
			bufferChannel.pop();
		}

	}
	//widget->unlockWidget();
	widget->unlockChannel();

	//UINFO("end try channel, buffer size %i",bufferChannel.size());
}

void NetworkVisualizer::onStartWeight()
{
	//UINFO("try weight");
	GuiWeightPtr weight = bufferWeight.front();

	if(widget->trylockWidget())
	{
		if(weight)
		{
			if(widget->updateEdge(weight))
				bufferWeight.pop();
		}
	}
	widget->unlockWidget();
	widget->unlockWeight();
	//UINFO("end try weight, buffer size %i",bufferWeight.size());
}

void NetworkVisualizer::onStartActivation()
{
	//UINFO("try activation");
	GuiActivationPtr activation = bufferActivation.front();

	//if(widget->trylockWidget())
	if(	activation )
	{
		widget->updateActivationValue(activation->getIndex(), activation->getValue(), activation->getIdLayer());
		bufferActivation.pop();
	}
	//widget->unlockWidget();
	mutActivation.unlock();
}

void NetworkVisualizer::onStartAnticipatedEvent()
{
	//UINFO("try anticipatedEvent");
	GuiAnticipatedEventPtr anticipatedEvent = bufferAnticipatedEvent.front();

	if(widget->trylockWidget())
		if(anticipatedEvent )
		{
			widget->updateAnticipatedEvent(anticipatedEvent->getIndex(), anticipatedEvent->getNewVigilance(),anticipatedEvent->getNewLearningRate(), anticipatedEvent->isAnticipated());
			bufferAnticipatedEvent.pop();
		}
	widget->unlockWidget();
	mutAnticipatedEvent.unlock();
}

int NetworkVisualizer::onChangeVigilance()
{
	QDoubleSpinBox *vigilanceSpinBox = (QDoubleSpinBox *)sender();
	int id = vigilanceSpinBox->property("id").toInt();

	//Emettre un signal
	emit changeVigilanceSignal(id, vigilanceSpinBox->value());

	widget->setVigilance(id,vigilanceSpinBox->value());

	return id;

}

int NetworkVisualizer::onChangeLearningRate()
{
	QDoubleSpinBox *learningRateSpinBox = (QDoubleSpinBox *)sender();
	int id = learningRateSpinBox->property("id").toInt();

	//Emettre un signal
	emit changeLearningRateSignal(id, learningRateSpinBox->value());

	widget->setLearningRate(id,learningRateSpinBox->value());

	return id;
}

int NetworkVisualizer::onChangeRelevance(double relevance)
{
	QDoubleSpinBox *relevanceSpinBox = (QDoubleSpinBox *)sender();
	int id = relevanceSpinBox->property("id").toInt();

	//Emettre un signal
	emit changeRelevanceSignal(id, relevance);

	widget->setRelevance(id,relevance);

	return id;
}

void NetworkVisualizer::onUpdateEmotionIntensity(int value)
{
	this->widget->setProgressBarValue(value);
}


void NetworkVisualizer::createActions()
{
	randomEvent_ = new QAction( tr("&Input random event"), this);
	randomEvent_->setStatusTip(tr("a new input event will be processed in the network"));
	connect(randomEvent_, SIGNAL(triggered()), this, SLOT(onMenuNewInput()));

	resetNetwork_ = new QAction( tr("Reset Episodic memory"), this);
	resetNetwork_->setStatusTip(tr("Inputs, channels, and weight are deleted from the database and the view"));
	connect(resetNetwork_, SIGNAL(triggered()), this, SLOT(onMenuResetNetwork()));

	clearActivation_ = new QAction(tr("Clear activation"), this);
	connect(clearActivation_,SIGNAL(triggered()),this,SLOT(onClearActivationButton()));

	toggleLearningMode_ = new QAction(tr("Episode learning mode"), this);
	connect(toggleLearningMode_,SIGNAL(triggered()),this,SLOT(onMenuToggleLearningMode()));
}

void NetworkVisualizer::createToolbar()
{
	actionToolbar = addToolBar(tr("Menu"));
	actionToolbar->addAction(randomEvent_);
	actionToolbar->addAction(resetNetwork_);
	actionToolbar->addAction(clearActivation_);
	actionToolbar->addAction(toggleLearningMode_);
}
