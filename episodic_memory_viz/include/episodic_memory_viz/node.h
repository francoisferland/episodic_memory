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

#ifndef NODE_H
#define NODE_H

#include <QGraphicsItem>
#include <QList>
#include <QLabel>
#include <boost/thread.hpp>
#include <boost/enable_shared_from_this.hpp>

class Edge;
class GraphWidget;
class QGraphicsSceneMouseEvent;

class Node;
typedef boost::shared_ptr<Node> NodePtr;

/**
 * This class represent a Node in the network viewer. A node can be an input or an output category
 */
class Node : public QGraphicsItem, public boost::enable_shared_from_this<Node>
{
public:
	/**
	 * Constructor
	 * @param graphWidget : pointer to the parent graphWidget
	 */
	Node(GraphWidget *graphWidget);
	/**
	 *	This function add an edge in the edgeUp list. The edge up list links the node to the upper layer
	 *	@param edge : a shared ptr to an edge object
	 */
	void addEdgeUp(EdgePtr edge);
	/**
	 *	This function add an edge in the edgeDown list. The edge down list links the node to the lower layer
	 *	@param edge : a shared ptr to an edge object
	 */
	void addEdgeDown(EdgePtr edge);
	/**
	 * Getter function, edgeUp list
	 * @return a QList that contains shared ptr to edge object
	 */
	QList<EdgePtr> edgesUp() const;
	/**
	 * Getter function, edgeDown list
	 * @return a QList that contains shared ptr to edge object
	 */
	QList<EdgePtr> edgesDown() const;
	/**
	 * This function defines the bounding rect around the graphic item
	 * @return QRectF object
	 */
	QRectF boundingRect() const;
	/**
	 * Returns the shape of this item (Ellipse) in local coordinates instead of calling a boundingRect
	 */
	QPainterPath shape() const;
	/**
	 * the paint function, draws all the Graphic items
	 */
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
	/**
	 * This function set the variable to toggle highlight of the node (red vs blue)
	 * the variable is later used in paint function
	 * @param isHighlight: highlight if true
	 */
	void hightlightNode(bool isHighlight);

	void showDescription(bool isDisplayed);
	/**
	 * This function clears all the edge connected to this node (reset shared pointers)
	 */
	void clearAllEdge();
	/**
	 * Getter function, is activated node
	 * @return true if node is activated, false otherwise
	 */
	bool isNodeActivated(){return activatedNode;}
	/**
	 * Setter function, id number
	 */
	void setIdNumber(int number){ idNumber = number;}
	/*
	 * Getter function, idNumber
	 */
	int getIdNumber(){return idNumber;}
	/**
	 * Setter function, description
	 */
	void setDescription(QString p_description){description = p_description;}
	/**
	 * Getter function, description
	 */
	QString getDescription(){return description;}
	/**
	 * Setter function, Activation value
	 */
	void setActivationValue(float value){activationValue = value;}
	/**
	 * Getter function, activation value
	 */
	double getActivationValue(){return activationValue;}

	void setComplement(bool is){isComplement_ = is;}
	bool isComplement(){return isComplement_;}

	bool isAnticipated() const {
		return isAnticipated_;
	}

	void setIsAnticipated(bool isAnticipated) {
		isAnticipated_ = isAnticipated;
	}

	void setStability(float stability) {
		stability_ = stability;
	}

protected:
	/**
	 * 	This function receives event when the mouse enter a node
	 */
	void hoverEnterEvent(QGraphicsSceneHoverEvent*);
	/**
	 * 	This function receives event when the mouse leave a node
	 */
	void hoverLeaveEvent(QGraphicsSceneHoverEvent*);
	/**
	 * 	This function receives event when the mouse clicks on the node
	 */
	void mousePressEvent(QGraphicsSceneMouseEvent * event);

private:

	QList<EdgePtr> edgeListUp;
	QList<EdgePtr> edgeListDown;
	GraphWidget *graph;
	QString description;
	int idNumber;
	float activationValue;
	bool m_hover;
	bool activatedNode;
	bool highlightToggle;
	bool displayDescription;
	bool isComplement_;
	bool isAnticipated_;
	//the more stability, the bigger the node
	float stability_;

};

#endif
