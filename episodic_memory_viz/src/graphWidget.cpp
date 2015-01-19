#include <graphWidget.h>
#include <edge.h>
#include <node.h>
#include <QtGui>
#include <math.h>

GraphWidget::GraphWidget(QWidget *parent)
: QGraphicsView(parent)
{
	QGraphicsScene *scene = new QGraphicsScene(this);
	scene->setItemIndexMethod(QGraphicsScene::NoIndex);
	scene->setSceneRect(-600, -600, 1280, 800);

	setScene(scene);

	emotionProgress_.setRange(0,100);
	emotionProgress_.setOrientation(Qt::Vertical);
	emotionProgress_.setValue(0);
	emotionProgress_.setTextVisible(false);
	emotionProgress_.setToolTip("Emotion Intensity");
	scene->addWidget(&emotionProgress_);

	setRenderHint(QPainter::Antialiasing);
	setTransformationAnchor(AnchorUnderMouse);
	//scale(qreal(1.2), qreal(1.2));
	setMinimumSize(1400, 900);
	setWindowTitle(tr("Episodic memory visualizer"));

	learningMode.setText("Recognizing");
	learningMode.setStyleSheet("QLabel { background-color : transparent; }");
	scene->addWidget(&learningMode);

	totalWidth_ = 0;
}

GraphWidget::~GraphWidget()
{
}

void GraphWidget::drawBackground(QPainter *painter, const QRectF &rect)
{
	mutDrawing.lock();

	Q_UNUSED(rect);

	QRectF sceneRect = this->sceneRect();

	int * offsetInput = new int;
	int * offsetEvent = new int;
	int * offsetEpisode = new int;
	int * offset = new int;

	//position of the emotion progressbar
	emotionProgress_.setGeometry(sceneRect.left()+2,sceneRect.top() + 10 ,20, sceneRect.height() - 80);

	QRectF rightShadow(sceneRect.right(), sceneRect.top() + 5, 5, sceneRect.height());
	QRectF bottomShadow(sceneRect.left() + 5, sceneRect.bottom(), sceneRect.width(), 5);
	if (rightShadow.intersects(rect) || rightShadow.contains(rect))
		painter->fillRect(rightShadow, Qt::darkGray);
	if (bottomShadow.intersects(rect) || bottomShadow.contains(rect))
		painter->fillRect(bottomShadow, Qt::darkGray);

	// Fill
	QLinearGradient gradient(sceneRect.topLeft(), sceneRect.bottomRight());
	gradient.setColorAt(0, Qt::white);
	gradient.setColorAt(1, Qt::lightGray);
	painter->fillRect(rect.intersect(sceneRect), gradient);
	painter->setBrush(Qt::NoBrush);

	int layerWidth = totalWidth_ > 1280 ? totalWidth_ : 1280;

	renderLayer(painter,layerWidth);

	(*offsetInput) = 0;
	(*offsetEvent) = 0;
	(*offsetEpisode) = 0;
	(*offset) = 0;

	totalWidth_ = 220;
	//display channel for each layer
	//if(mutChannelRender.try_lock())
	{
		//UINFO("Render channel");
		for(std::map<int,GuiChannelPtr>::iterator it = mapGuiChannel.begin() ; it != mapGuiChannel.end() ; it++)
		{
			if((*it).second->getLayer() == EM_GUI::INPUT_LAYER)
			{
				offset = offsetInput;
			}
			else if((*it).second->getLayer() ==  EM_GUI::EVENT_LAYER)
			{
				offset = offsetEvent;
			}
			else if((*it).second->getLayer() == EM_GUI::EPISODE_LAYER)
			{
				offset = offsetEpisode;
			}

			int width = 0;
			if((*it).second->getLayer() == EM_GUI::INPUT_LAYER)
				width = getListCategoryByChannel((*it).second->getChannelId()).size() * NEURON_WIDTH ;
			else
				width = getListCategoryByChannel((*it).second->getChannelId()).size() * (NEURON_WIDTH + 20);

			QFont myFont = painter->font();
			myFont.setPointSize(12);
			std::string description = (*it).second->getDescription().toStdString();
			QString labelDesc(description.c_str());

			QFontMetrics fm(myFont);
			int minimumWidth=fm.width(labelDesc)+110;

			if(width < minimumWidth)
				width = minimumWidth;

			if( (*it).second->getLayer() == 1 )
				totalWidth_ += width;

			renderChannel(painter ,*offset, width, (*it).second);
			//calculate offset
			(*offset) += width ;
		}
		//UINFO("Gui end render channel");
	}
	//mutChannelRender.unlock();

	if(totalWidth_ > sceneRect.width() && totalWidth_ > 1280)
	{
		this->scene()->setSceneRect(-600, -600, totalWidth_, 800);
	}

	painter->drawRect(sceneRect);

	mutDrawing.unlock();
}


void GraphWidget::renderLayer(QPainter * painter , int width)
{
	QRectF sceneRect = this->sceneRect();

	episodeLayer = new QRectF(sceneRect.left() + 25,  sceneRect.top() + 5,                       width - 10, sceneRect.height()/4);
	eventLayer =   new QRectF(sceneRect.left() + 25, (sceneRect.top() + sceneRect.height()/3),   width - 10, sceneRect.height()/4);
	inputLayer =   new QRectF(sceneRect.left() + 25, (sceneRect.top() + 2*sceneRect.height()/3), width - 10, sceneRect.height()/4);

	painter->fillRect(*episodeLayer, Qt::darkGray);
	painter->fillRect(*eventLayer, Qt::darkGray);
	painter->fillRect(*inputLayer, Qt::darkGray);

	QRectF textRectEpisode(episodeLayer->left() + 5, episodeLayer->top() + 4, episodeLayer->width() - 4, episodeLayer->height() - 4);
	QString messageEpisode(tr("Episode Layer"));
	QRectF textRectEvent(eventLayer->left() + 5, eventLayer->top() + 4, eventLayer->width() - 4, eventLayer->height() - 4);
	QString messageEvent(tr("Event Layer"));
	QRectF textRectInput(inputLayer->left() + 5, inputLayer->top() + 4, inputLayer->width() - 4, inputLayer->height() - 4);
	QString messageInput(tr("Input Layer"));

	QFont font = painter->font();
	font.setPointSize(12);
	painter->setFont(font);

	QRectF textRectVigilance(textRectEvent.left() + 115, textRectEvent.bottom() - 180 , 150, 20);
	painter->drawText(textRectVigilance, "Vigilance");

	QRectF textRectLearningRate(textRectEvent.left() + 85, textRectEvent.bottom() - 75 , 150, 20);
	painter->drawText(textRectLearningRate, "LearningRate");

	font.setPointSize(18);
	QRectF textRectLearningMode(textRectEpisode.left() + 30, textRectEpisode.bottom() - 100 , 150, 30);
	learningMode.setFont(font);
	learningMode.setGeometry(textRectLearningMode.left() , textRectLearningMode.bottom() , textRectLearningMode.width(), textRectLearningMode.height());

	font.setBold(true);
	font.setPointSize(14);
	painter->setFont(font);
	painter->setPen(Qt::lightGray);
	painter->drawText(textRectEpisode.translated(2, 2), messageEpisode);
	painter->drawText(textRectEvent.translated(2, 2), messageEvent);
	painter->drawText(textRectInput.translated(2, 2), messageInput);
	painter->setPen(Qt::black);
	painter->drawText(textRectEpisode, messageEpisode);
	painter->drawText(textRectEvent, messageEvent);
	painter->drawText(textRectInput, messageInput);

}

void GraphWidget::renderChannel(QPainter * painter, int offset, int width, GuiChannelPtr channel)
{
	//ROS_DEBUG("inside addChannel");

	if(!channel)
	{
		ROS_WARN("channel does not exist");
		return;
	}
	double relevance = channel->getRelevance();
	QString description = channel->getDescription();
	QDoubleSpinBox* relevanceSpinBox = mapRelevanceSpinBox[channel->getChannelId()];

	QRectF* channelRect = NULL;
	QRectF* layerRect = NULL;
	QFont font = painter->font();
	font.setBold(false);
	font.setPointSize(8);
	painter->setFont(font);
	painter->setPen(Qt::black);

	int offsetNode = 0;

	switch(channel->getLayer())
	{
	case EM_GUI::INPUT_LAYER:
	{
		layerRect = inputLayer;
		offsetNode = NEURON_WIDTH;
		break;
	}
	case EM_GUI::EVENT_LAYER:
	{
		layerRect = eventLayer;
		offsetNode = NEURON_WIDTH + 20;
		break;
	}
	case EM_GUI::EPISODE_LAYER:
	{
		layerRect = episodeLayer;
		offsetNode = NEURON_WIDTH + 20;
		break;
	}
	}

	channelRect = new QRectF( layerRect->left() + 220 + offset, layerRect->top() + 5, width - 20, layerRect->height() - 10);
	painter->setOpacity(0.5);
	painter->fillRect(*channelRect, Qt::lightGray);

	//add the nodes
	offset = NEURON_WIDTH/2 - 10;

	//find the node related to this channel
	std::map<int,GuiCategoryPtr> mapCategory = getListCategoryByChannel(channel->getChannelId());
	//select the position of the node
	for(std::map<int,GuiCategoryPtr>::iterator it = mapCategory.begin() ; it != mapCategory.end() ; it++)
	{
		renderNode(channelRect->left() + offset , channelRect->top() + channelRect->height()/2 -10, (*it).second->getIndex(), (*it).second->getLayerId());
		offset += offsetNode;
	}

	//add the channel description
	QRectF textRect(channelRect->left() + 4, channelRect->bottom() - 25 , channelRect->width() - 8, 20);
	font.setPointSize(12);
	painter->setFont(font);
	painter->drawText(textRect, description);

	//add the relevance parameter
	QRectF relevanceRect(textRect.left(), textRect.top() - textRect.height(), textRect.width(), textRect.height());
	font.setPointSize(10);
	painter->setFont(font);
	painter->drawText(relevanceRect,Qt::AlignVCenter, "relevance : ");

	if(relevanceSpinBox)
	{
		relevanceSpinBox->setGeometry(QRect(relevanceRect.topLeft().toPoint(),relevanceRect.bottomRight().toPoint()).translated(relevanceRect.width() - 55,0));
		relevanceSpinBox->setMaximumWidth(55);
		relevanceSpinBox->setMaximumHeight(relevanceRect.height()-2);
		if( round(relevanceSpinBox->value()) != round(relevance)) //prevent to refresh the gui each time
			relevanceSpinBox->setValue(relevance);
	}

	if(channelRect)
		delete channelRect;
}

void GraphWidget::renderNode(int posX, int posY, int idNumber, int layer)
{
	NodePtr node;
	switch(layer)
	{
	case 1: //input layer
	{
		node = listNodeInputLayer[idNumber];
		break;
	}
	case 2:// event layer
	{
		node = listNodeEventLayer[idNumber];
		//place the vigilance and learning rate spinbox
		QDoubleSpinBox* vigilanceSpinBox = getVigilanceSpinbox(idNumber);
		vigilanceSpinBox->setGeometry(QRect(posX - 10, posY - 75, 55, 20));
		vigilanceSpinBox->setMaximumWidth(55);
		vigilanceSpinBox->setMaximumHeight(20);
		if( round(vigilanceSpinBox->value()) != round(mapGuiCategory[idNumber]->getVigilance()))
			vigilanceSpinBox->setValue(mapGuiCategory[idNumber]->getVigilance());

		QDoubleSpinBox* learningRateSpinBox = getLearningRateSpinbox(idNumber);
		learningRateSpinBox->setGeometry(QRect(posX - 10, posY + 30, 55, 20));
		learningRateSpinBox->setMaximumWidth(55);
		learningRateSpinBox->setMaximumHeight(20);
		if( round(learningRateSpinBox->value()) != round(mapGuiCategory[idNumber]->getLearningRate()))
			learningRateSpinBox->setValue(mapGuiCategory[idNumber]->getLearningRate());

		break;
	}
	case 3:// episodeLayer
	{
		node = listNodeEpisodeLayer[idNumber];

		QDoubleSpinBox* vigilanceSpinBox = getVigilanceSpinbox(idNumber);
		vigilanceSpinBox->setGeometry(QRect(posX - 10, posY - 75, 55, 20));
		vigilanceSpinBox->setMaximumWidth(55);
		vigilanceSpinBox->setMaximumHeight(20);
		if( round(vigilanceSpinBox->value()) != round(mapGuiCategory[idNumber]->getVigilance()))
			vigilanceSpinBox->setValue(mapGuiCategory[idNumber]->getVigilance());

		QDoubleSpinBox* learningRateSpinBox = getLearningRateSpinbox(idNumber);
		learningRateSpinBox->setGeometry(QRect(posX - 10, posY + 30, 55, 20));
		learningRateSpinBox->setMaximumWidth(55);
		learningRateSpinBox->setMaximumHeight(20);
		if( round(learningRateSpinBox->value()) != round(mapGuiCategory[idNumber]->getLearningRate()))
			learningRateSpinBox->setValue(mapGuiCategory[idNumber]->getLearningRate());

		break;
	}
	}

	if(!node)
	{
		ROS_ERROR("Can't render node id %i, please close application",idNumber);
		return;
	}

	node->setPos(posX, posY);
}

bool GraphWidget::addCategory(GuiCategoryPtr category)
{
	bool success = true;

	//first verify that a channel container is already instanciated
	success = (mapGuiChannel.count(category->getChannelId()) != 0);

	if(success)
	{

		NodePtr node = NodePtr(new Node(this));
		this->scene()->addItem(node.get());
		//node->setCacheMode(QGraphicsItem::ItemCoordinateCache);
		node->setIdNumber(category->getIndex());
		node->setActivationValue(0);
		if(!category->getDescription().isEmpty() && category->getDescription().startsWith('~') )
			node->setComplement(true);
		else
			node->setComplement(false);

		if(mapGuiCategory.count(category->getIndex()))
		{
			ROS_WARN("Category %i already exist", category->getIndex());
		}

		switch(category->getLayerId())
		{
		case EM_GUI::INPUT_LAYER:
		{
			node->setDescription(category->getDescription());
			this->listNodeInputLayer[node->getIdNumber()] = node;
			break;
		}

		case EM_GUI::EVENT_LAYER:
		{

			this->listNodeEventLayer[node->getIdNumber()] = node;
			//add vigilance spinBox and learningrate spinbox
			addNewLearningRateSpinBox(category->getIndex());
			addNewVigilanceSpinBox(category->getIndex());

			break;
		}

		case EM_GUI::EPISODE_LAYER:
		{
			this->listNodeEpisodeLayer[node->getIdNumber()] = node;
			//add vigilance spinBox and learningrate spinbox
			addNewLearningRateSpinBox(category->getIndex());
			addNewVigilanceSpinBox(category->getIndex());
			this->listNodeEpisodeLayer[node->getIdNumber()]->setStability(((0.25 - (category->getVigilance() - 0.60)) / (0.85 - 0.60)) );
			break;
		}
		}

		mapGuiCategory[category->getIndex()] = category;

		this->scene()->update();

		//adjust each weight
		for(std::map<int, NodePtr>::iterator it = this->listNodeEventLayer.begin(); it != this->listNodeEventLayer.end(); it++)
		{
			//update edge
			foreach(EdgePtr edge, (*it).second->edgesUp())
			{
				edge->adjust();
			}
			foreach(EdgePtr edge, (*it).second->edgesDown())
			{
				edge->adjust();
			}
		}
	}
	return success;

}


bool GraphWidget::addChannel(GuiChannelPtr channel)
{
	bool success = true;

	if(mapGuiChannel.count(channel->getChannelId()))
	{
		ROS_WARN("channel id already exist %i , %s", channel->getChannelId(), channel->getDescription().toStdString().c_str());
	}
	else
	{
		addNewRelevanceSpinBox(channel->getChannelId());
	}

	mapGuiChannel[channel->getChannelId()] = channel;

	return success;
}


void GraphWidget::addNewVigilanceSpinBox(int categoryId)
{
	QDoubleSpinBox * vigilanceSpinBox;
	vigilanceSpinBox = new QDoubleSpinBox();
	vigilanceSpinBox->setRange(0, 1);
	vigilanceSpinBox->setSingleStep(0.05);
	vigilanceSpinBox->setProperty("id",QVariant(categoryId));
	vigilanceSpinBox->setAccessibleName("V"+categoryId);
	if(this->scene())
	{
		mapProxyWidgetVigilance[categoryId] = this->scene()->addWidget(vigilanceSpinBox);
	}
	this->mapVigilanceSpinBox[categoryId] = vigilanceSpinBox;
}

void GraphWidget::addNewLearningRateSpinBox(int categoryId)
{
	QDoubleSpinBox * learningRateSpinBox;

	learningRateSpinBox= new QDoubleSpinBox();
	learningRateSpinBox->setRange(0, 1);
	learningRateSpinBox->setSingleStep(0.05);
	learningRateSpinBox->setProperty("id",QVariant(categoryId));
	learningRateSpinBox->setAccessibleName("L"+categoryId);
	if(this->scene())
	{
		mapProxyWidgetLearningRate[categoryId] = this->scene()->addWidget(learningRateSpinBox);
	}
	this->mapLearningRateSpinBox[categoryId] = learningRateSpinBox;
}

void GraphWidget::addNewRelevanceSpinBox(int channelId)
{
	QDoubleSpinBox * relevanceSpinBox;

	relevanceSpinBox = new QDoubleSpinBox();
	relevanceSpinBox->setRange(0, 1);
	relevanceSpinBox->setSingleStep(0.05);
	relevanceSpinBox->setProperty("id",QVariant(channelId));
	relevanceSpinBox->setAccessibleName("R"+channelId);
	if(this->scene())
	{
		mapProxyWidgetRelevance[channelId] = this->scene()->addWidget(relevanceSpinBox);
	}
	mapRelevanceSpinBox[channelId] = relevanceSpinBox;
}

bool GraphWidget::updateEdge(GuiWeightPtr weight)
{
	//UTimer timer;
	bool success = true;

	if(!weight)
	{
		return false;
		ROS_INFO("Error weight");
	}

	std::map<int, NodePtr>  listNodeUpper;
	std::map<int, NodePtr>  listNodeLower;
	switch(weight->getIdLayer())
	{
	case EM_GUI::INPUT_LAYER:
		listNodeLower = listNodeInputLayer;
		listNodeUpper = listNodeEventLayer;
		break;
	case EM_GUI::EVENT_LAYER:
		listNodeUpper = listNodeEpisodeLayer;
		listNodeLower = listNodeEventLayer;
		break;
	case EM_GUI::EPISODE_LAYER:
		listNodeLower = listNodeEpisodeLayer;
		//no upper layer...
		break;
	}

	//verify that the two nodes to connect exist
	if(listNodeUpper.count(weight->getIndexTopCategory()) && listNodeLower.count(weight->getIndexBottomLayer()))
	{
		if(!listNodeLower[weight->getIndexBottomLayer()]->isComplement())
		{
			//verify if the weight exists
			if(mapGuiWeight.count(weight->getId()) == 0)
			{
				//create a edge
				EdgePtr edge = EdgePtr(new Edge(listNodeLower[weight->getIndexBottomLayer()].get(), listNodeUpper[weight->getIndexTopCategory()].get()));
				//edge->setCacheMode(QGraphicsItem::ItemCoordinateCache);
				edge->init();
				edge->setlayer(weight->getIdLayer());//each edge needs to know if it is between input and event layer, or event and episode layer
				edge->setWeightValue(weight->getValue());
				edge->setOpacity(weight->getValue());
				this->scene()->addItem(edge.get());

				mapGuiWeight[weight->getId()] = weight;
			}
			else
			{
				//update the edge
				mapGuiWeight[weight->getId()]->setValue(weight->getValue());
				if(EdgePtr edge = isNodesConnected(listNodeLower[weight->getIndexBottomLayer()],listNodeUpper[weight->getIndexTopCategory()]))
				{
					edge->setOpacity(weight->getValue());
					edge->setWeightValue(weight->getValue());
				}
			}
		}
		else
		{
			ROS_INFO("skip complement node %i",weight->getId());
		}
	}
	else
	{
		ROS_WARN("The two nodes to connect does not exist up%i down%i",weight->getIndexTopCategory(),weight->getIndexBottomLayer());
		success = false;
	}

	//UWARN(" %f s", timer.ticks());
	return success;
}

bool GraphWidget::updateActivationValue(int index, float activationValue, int layer)
{
	std::map<int, NodePtr>  listNode;
	switch(layer)
	{
	case EM_GUI::INPUT_LAYER:
		listNode = listNodeInputLayer;
		break;
	case EM_GUI::EVENT_LAYER:
		listNode = listNodeEventLayer;
		break;
	case EM_GUI::EPISODE_LAYER:
		listNode = listNodeEpisodeLayer;
		break;
	}
	if(listNode.count(index) == 0)
	{
		ROS_WARN("The neuron to activate does not exist in the view. Layer %i, index %i",layer,index);
		return false;
	}

	if(layer != EM_GUI::INPUT_LAYER && activationValue == 1)
	{
		highlightWinningNode(index, listNode);
	}
	else if(layer == EM_GUI::INPUT_LAYER  && ( !listNode[index]->isComplement() ))
	{
		if( activationValue > 0)
			listNode[index]->showDescription(true);
		else
			listNode[index]->showDescription(false);
	}

	listNode[index]->setActivationValue(activationValue);
	//mutDrawing.lock();
	listNode[index]->update();
	//mutDrawing.unlock();
	return true;
}

bool GraphWidget::updateAnticipatedEvent(int index, float newVigilance, float newLearningRate, bool anticipated)
{
	bool success = true;

	//change the vigilance and learningRate
	success &= mapVigilanceSpinBox.count(index) == 1;
	success &= mapLearningRateSpinBox.count(index) == 1;
	success &= mapGuiCategory.count(index) == 1;

	std::map<int,NodePtr> listNode;
	if(listNodeEventLayer.count(index) == 1)
		listNode = listNodeEventLayer;
	else if(listNodeEpisodeLayer.count(index) == 1)
	{
		listNode = listNodeEpisodeLayer;
		//in this case, we are updating vigilance and learning rate of the episode
		if (success)
			listNode[index]->setStability(((0.25 - (newVigilance - 0.60)) / (0.85 - 0.60)) );
	}
	else
		success = false;

	if(success)
	{
		mapVigilanceSpinBox[index]->setValue(newVigilance);
		mapLearningRateSpinBox[index]->setValue(newLearningRate);
		mapGuiCategory[index]->setVigilance(newVigilance);
		listNode[index]->setIsAnticipated(anticipated);
		//listNodeEventLayer[index]->update();
	}
	else
		ROS_WARN("can't find anticipated node %i",index);

	return success;
}

EdgePtr GraphWidget::isNodesConnected(NodePtr upperNode, NodePtr lowerNode)
{
	EdgePtr connection;

	foreach(EdgePtr edge, upperNode->edgesDown())
	{
		if(((Node*)edge->getDest())->getIdNumber() == lowerNode->getIdNumber())
		{
			connection = edge;
			break;
		}
	}

	return connection;
}

std::map<int,GuiCategoryPtr> GraphWidget::getListCategoryByChannel(int channelId)
{
	std::map<int,GuiCategoryPtr> mapCategory;

	for(std::map<int,GuiCategoryPtr>::iterator it = mapGuiCategory.begin() ; it != mapGuiCategory.end() ; it++)
	{
		if((*it).second->getChannelId() == channelId)
		{
			mapCategory[(*it).second->getIndex()] = (*it).second;
		}
	}
	return mapCategory;
}

QDoubleSpinBox* GraphWidget::getVigilanceSpinbox(int categoryId)
{
	QDoubleSpinBox* spinbox = NULL;

	if(mapVigilanceSpinBox.count(categoryId))
	{
		spinbox = mapVigilanceSpinBox[categoryId];
	}

	return spinbox;
}

QDoubleSpinBox* GraphWidget::getLearningRateSpinbox(int categoryId)
{
	QDoubleSpinBox* spinbox = NULL;

	if(mapLearningRateSpinBox.count(categoryId))
	{
		spinbox = mapLearningRateSpinBox[categoryId];
	}

	return spinbox;
}

QDoubleSpinBox* GraphWidget::getRelevanceSpinbox(int channelId)
{
	QDoubleSpinBox* spinbox = NULL;

	if(mapRelevanceSpinBox.count(channelId))
	{
		spinbox = mapRelevanceSpinBox[channelId];
	}

	return spinbox;
}

void GraphWidget::highlightWinningNode(int index, std::map<int,NodePtr> map)
{
	//turn off all the nodes
	for(std::map<int,NodePtr>::iterator it = map.begin(); it != map.end() ; it++)
	{
		(*it).second->hightlightNode(false);
	}
	//highlight the winning node
	if(map.count(index))
	{
		map[index]->hightlightNode(true);
	}

}

void GraphWidget::clearHighlightNode(int layer)
{
	std::map<int,NodePtr> map;
	switch(layer)
	{
	case EM_GUI::INPUT_LAYER:
		map = this->listNodeInputLayer;
		break;

	case EM_GUI::EVENT_LAYER:
		map = this->listNodeEventLayer;
		break;

	case EM_GUI::EPISODE_LAYER:
		map = this->listNodeEpisodeLayer;
		break;

	default:
		break;
	}

	for(std::map<int,NodePtr>::iterator it = map.begin(); it != map.end() ; it++)
	{
		(*it).second->hightlightNode(false);
	}
}

void GraphWidget::keyPressEvent(QKeyEvent *event)
{
	switch (event->key()) {
	case Qt::Key_Plus:
		zoomIn();
		break;
	case Qt::Key_Minus:
		zoomOut();
		break;
	default:
		QGraphicsView::keyPressEvent(event);
		break;
	}
}

void GraphWidget::setLearningRate(int categoryId, float learningRate)
{
	this->mapGuiCategory[categoryId]->setLearningRate(learningRate);
}

void GraphWidget::setVigilance(int categoryId, float vigilance)
{
	this->mapGuiCategory[categoryId]->setVigilance(vigilance);
}

void GraphWidget::setRelevance(int channelId, float relevance)
{
	this->mapGuiChannel[channelId]->setRelevance(relevance);
}

void GraphWidget::setProgressBarValue(int value)
{
	emotionProgress_.setValue(value);
}

void GraphWidget::clear()
{
	//clear edge
	clearAllEdge();
	for(std::map<int,GuiWeightPtr>::iterator it = mapGuiWeight.begin() ; it != mapGuiWeight.end() ; it++)
	{
		if((*it).second)
		{
			(*it).second.reset();
		}
	}
	mapGuiWeight.clear();

	//clear node
	clearLayerNode(&this->listNodeEventLayer);
	clearLayerNode(&this->listNodeEpisodeLayer);
	clearLayerNode(&this->listNodeInputLayer);
	for(std::map<int,GuiCategoryPtr>::iterator it = mapGuiCategory.begin() ; it != mapGuiCategory.end() ; it++)
	{
		if((*it).second)
		{
			(*it).second.reset();
		}
	}
	mapGuiCategory.clear();

	//clear channel
	for(std::map<int,GuiChannelPtr>::iterator it = mapGuiChannel.begin() ; it != mapGuiChannel.end() ; it++)
	{
		if((*it).second)
		{
			(*it).second.reset();
		}
	}
	mapGuiChannel.clear();


	for(std::map<int,QGraphicsProxyWidget*>::iterator it = mapProxyWidgetVigilance.begin() ; it != mapProxyWidgetVigilance.end() ; it++)
	{
		QGraphicsProxyWidget * temp = (*it).second;
		if(temp)
		{
			delete temp;
			temp = 0;
		}
	}
	for(std::map<int,QGraphicsProxyWidget*>::iterator it = mapProxyWidgetLearningRate.begin() ; it != mapProxyWidgetLearningRate.end() ; it++)
	{
		QGraphicsProxyWidget * temp = (*it).second;
		if(temp)
		{
			delete temp;
			temp = 0;
		}
	}
	for(std::map<int,QGraphicsProxyWidget*>::iterator it = mapProxyWidgetRelevance.begin() ; it != mapProxyWidgetRelevance.end() ; it++)
	{
		QGraphicsProxyWidget * temp = (*it).second;
		if(temp)
		{
			delete temp;
			temp = 0;
		}
	}

	mapVigilanceSpinBox.clear();
	mapLearningRateSpinBox.clear();
	mapRelevanceSpinBox.clear();
	mapProxyWidgetVigilance.clear();
	mapProxyWidgetLearningRate.clear();
	mapProxyWidgetRelevance.clear();

	this->scene()->update();
}

void GraphWidget::clearAllActivation()
{
	for(std::map<int,NodePtr>::iterator it = listNodeInputLayer.begin() ; it != listNodeInputLayer.end() ; it++)
	{
		updateActivationValue((*it).first, 0, EM_GUI::INPUT_LAYER);
	}
	for(std::map<int,NodePtr>::iterator it = listNodeEventLayer.begin() ; it != listNodeEventLayer.end() ; it++)
	{
		updateActivationValue((*it).first, 0, EM_GUI::EVENT_LAYER);
		(*it).second->setIsAnticipated(false);
	}
	for(std::map<int,NodePtr>::iterator it = listNodeEpisodeLayer.begin() ; it != listNodeEpisodeLayer.end() ; it++)
	{
		updateActivationValue((*it).first, 0, EM_GUI::EPISODE_LAYER);
	}
}

void GraphWidget::clearAllEdge()
{
	for(std::map<int, NodePtr>::iterator it= this->listNodeInputLayer.begin(); it != listNodeInputLayer.end(); it++)
	{
		(*it).second->clearAllEdge();
	}
	for(std::map<int, NodePtr>::iterator it= this->listNodeEventLayer.begin(); it != listNodeEventLayer.end(); it++)
	{
		(*it).second->clearAllEdge();
	}
	for(std::map<int, NodePtr>::iterator it= this->listNodeEpisodeLayer.begin(); it != listNodeEpisodeLayer.end(); it++)
	{
		(*it).second->clearAllEdge();
	}

	mapGuiWeight.clear();
}

void GraphWidget::clearLayerNode(std::map<int, NodePtr>* layer)
{
	for(std::map<int, NodePtr>::iterator it= layer->begin(); it != layer->end(); it++)
	{
		this->scene()->removeItem((*it).second.get());
		(*it).second.reset();
	}
	layer->clear();
}

void GraphWidget::wheelEvent(QWheelEvent *event)
{
	scaleView(pow((double)2, event->delta() / 240.0));
}

void GraphWidget::scaleView(qreal scaleFactor)
{
	qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
	if (factor < 0.07 || factor > 100)
		return;

	scale(scaleFactor, scaleFactor);
}

void GraphWidget::zoomIn()
{
	scaleView(qreal(1.2));
}

void GraphWidget::zoomOut()
{
	scaleView(1 / qreal(1.2));

}

