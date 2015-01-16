/****************************************************************************
 **
 ** Copyright (C) 2012 Nokia Corporation and/or its subsidiary(-ies).
 ** All rights reserved.
 ** Contact: Nokia Corporation (qt-info@nokia.com)
 **
 ** This file is part of the examples of the Qt Toolkit.
 **
 ** $QT_BEGIN_LICENSE:BSD$
 ** You may use this file under the terms of the BSD license as follows:
 **
 ** "Redistribution and use in source and binary forms, with or without
 ** modification, are permitted provided that the following conditions are
 ** met:
 **   * Redistributions of source code must retain the above copyright
 **     notice, this list of conditions and the following disclaimer.
 **   * Redistributions in binary form must reproduce the above copyright
 **     notice, this list of conditions and the following disclaimer in
 **     the documentation and/or other materials provided with the
 **     distribution.
 **   * Neither the name of Nokia Corporation and its Subsidiary(-ies) nor
 **     the names of its contributors may be used to endorse or promote
 **     products derived from this software without specific prior written
 **     permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 ** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 ** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 ** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 ** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 ** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 ** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 ** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 ** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 ** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 ** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
 ** $QT_END_LICENSE$
 **
 ****************************************************************************/

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOption>

#include <edge.h>
#include <node.h>
#include <graphWidget.h>

Node::Node(GraphWidget *graphWidget)
: graph(graphWidget)
{
	setFlag(ItemIsFocusable);
	setFlag(ItemSendsGeometryChanges);
	setAcceptHoverEvents(true);
	setCacheMode(DeviceCoordinateCache);
	setZValue(-1);
	m_hover = false;
	description = "";
	idNumber = -1;
	activationValue = 0;
	activatedNode = false;
	highlightToggle = false;
	displayDescription = false;
	isComplement_ = false;
	isAnticipated_ = false;
	stability_ = 0;
}

void Node::addEdgeUp(EdgePtr edge)
{
	edgeListUp << edge;
	edge->adjust();
}

void Node::addEdgeDown(EdgePtr edge)
{
	edgeListDown << edge;
	edge->adjust();
}

QList<EdgePtr> Node::edgesUp() const
{
	return edgeListUp;
}

QList<EdgePtr> Node::edgesDown() const
{
	return edgeListDown;
}

void Node::clearAllEdge()
{
	for(QList<EdgePtr>::iterator it = edgeListDown.begin() ; it != edgeListDown.end() ; it++)
	{
		if((*it))
		{
			(*it).reset();
		}
	}
	edgeListDown.clear();

	for(QList<EdgePtr>::iterator it = edgeListUp.begin() ; it != edgeListUp.end() ; it++)
	{
		if((*it))
		{
			(*it).reset();
		}
	}
	edgeListUp.clear();
}

QRectF Node::boundingRect() const
{
	qreal adjust = 2;
	return QRectF( -60 - adjust, -60 - adjust,
			120 + adjust, 120 + adjust);
}

QPainterPath Node::shape() const
{
	QPainterPath path;
	int minDiam = 20;
	int maxDiam = 60;
	int diam = minDiam + (maxDiam - minDiam) * stability_;
	path.addEllipse(-diam/2, -diam/2, diam, diam);
	return path;
}

void Node::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget * widget)
{
	int minDiam = 20;
	int maxDiam = 60;
	int diam = minDiam + (maxDiam - minDiam) * stability_;

	painter->setPen(Qt::NoPen);
	painter->setBrush(Qt::darkGray);
	painter->drawEllipse(-diam/2, -diam/2, diam, diam);

	foreach(EdgePtr  edge, this->edgesUp())
	{
		edge->changeOnSelected((m_hover||highlightToggle));
	}
	foreach(EdgePtr edge, this->edgesDown())
	{
		edge->changeOnSelected((m_hover||highlightToggle));
	}

	QRadialGradient gradient(-3, -3, diam/2);
	gradient.setCenter(3, 3);
	gradient.setFocalPoint(3, 3);

	if(isAnticipated_)
	{
		gradient.setColorAt(1, QColor(Qt::green).light(120));
		gradient.setColorAt(0, QColor(Qt::yellow).light(120));
		painter->setBrush(gradient);
		painter->drawRoundRect(QRectF(-15,-20,diam+10,diam+10),10,10);
	}

	if (option->state & QStyle::State_Sunken) {

		if(!(m_hover||highlightToggle))
		{
			if(activatedNode)
			{
				gradient.setColorAt(1, QColor(Qt::red).light(120));
				gradient.setColorAt(0, QColor(Qt::darkRed).light(120));
			}
			else
			{
				gradient.setColorAt(1, QColor(Qt::cyan).light(120));
				gradient.setColorAt(0, QColor(Qt::blue).light(120));
			}
		}else
		{
			gradient.setColorAt(1, QColor(Qt::yellow).light(120));
			gradient.setColorAt(0, QColor(Qt::darkYellow).light(120));
		}

	} else {
		gradient.setCenter(0, 0);
		gradient.setFocalPoint(0, 0);
		if(!(m_hover||highlightToggle))
		{
			if(activatedNode)
			{
				gradient.setColorAt(0, Qt::red);
				gradient.setColorAt(1, Qt::darkRed);
			}
			else
			{
				gradient.setColorAt(0, Qt::cyan);
				gradient.setColorAt(1, Qt::blue);
			}
		}else
		{
			gradient.setColorAt(0, Qt::yellow);
			gradient.setColorAt(1, Qt::darkYellow);
		}
	}

	painter->setBrush(gradient);

	painter->setPen(QPen(Qt::black, 0));

	painter->drawEllipse(-diam/2, -diam/2, diam, diam);

	//paint the idNumber next to the ellipse
	QFont font = painter->font();
	font.setBold(false);
	font.setPointSize(5);
	painter->setFont(font);
	painter->setPen(Qt::black);
	QRectF textRect(boundingRect().left()+45 , boundingRect().bottom() -50 , boundingRect().width(), boundingRect().height() - 4);
	painter->drawText(textRect, QString::number(getIdNumber(),10));


	//paint the description if there's one
	if(m_hover || highlightToggle || displayDescription)
	{
		QRectF descRect(boundingRect().left() , boundingRect().bottom()-30  , boundingRect().width()+40, boundingRect().height()+10 );
		font.setPointSize(14);
		painter->setFont(font);
		painter->setPen(Qt::darkRed);
		painter->drawText(descRect, getDescription(), QTextOption(Qt::AlignHCenter));
	}

	if(getActivationValue())
	{
		//paint the activation value on top of the node
		QRectF activationRec(boundingRect().center(), QSize(6, -getActivationValue()*40) );
		painter->setPen(QPen(Qt::black,0));
		painter->setBrush(Qt::cyan);
		painter->drawRect((activationRec.translated(-2,-9)));
	}

}

void Node::hoverEnterEvent(QGraphicsSceneHoverEvent* event)
{
	//highlight the current node and edges
	m_hover = true;
	this->setZValue(101);
	update();
	QGraphicsItem::hoverEnterEvent(event);

	//show description of the connected nodes
	foreach(EdgePtr edge, this->edgesUp())
	{
		if(edge->getWeightValue() > 0)
			edge->forceDisplayDescription(true);
	}

	foreach(EdgePtr edge, this->edgesDown())
	{
		if(edge->getWeightValue() > 0)
			edge->forceDisplayDescription(true);
	}

	//	if(activatedNode && graph->findNodeEpisode(shared_from_this()))
	//	{
	//		graph->getEpisodeDetail()->QWidget::show();
	//	}
}

void Node::hoverLeaveEvent(QGraphicsSceneHoverEvent* event)
{
	//reset the current node and edges
	m_hover = false;
	this->setZValue(5);
	update();
	QGraphicsItem::hoverLeaveEvent(event);

	//show description of the connected nodes
	foreach(EdgePtr edge, this->edgesUp())
	{
		if(edge->getWeightValue() > 0)
			edge->forceDisplayDescription(false);
	}
	foreach(EdgePtr edge, this->edgesDown())
	{
		if(edge->getWeightValue() > 0)
			edge->forceDisplayDescription(false);
	}

	//	if(activatedNode && !highlightToggle){
	//		graph->getEpisodeDetail()->QWidget::hide();
	//	}
}

void Node::mousePressEvent(QGraphicsSceneMouseEvent * event)
{
	highlightToggle = !highlightToggle;
	foreach(EdgePtr  edge, this->edgesUp())
	{
		edge->lockColor(highlightToggle);
	}
	foreach(EdgePtr edge, this->edgesDown())
	{
		edge->lockColor(highlightToggle);
	}
}

void Node::hightlightNode(bool isActivated)
{
	activatedNode = isActivated;
	update();
}

void Node::showDescription(bool isDisplayed)
{
	displayDescription = isDisplayed;
	update();
}

