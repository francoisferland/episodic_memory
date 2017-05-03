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

#ifndef EDGE_H
#define EDGE_H

#include <QGraphicsItem>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <graphWidget.h>

static int positionWeightValue = 0;


class Edge;
typedef boost::shared_ptr<Edge> EdgePtr;
class Node;

/**
 *	This class represent a graphic object that connect nodes between each other.
 *	In the ART network, it referred as a weight
 *	They can have a different color when the mouse hover the connected node
 */
class Edge : public boost::enable_shared_from_this<Edge> , public QGraphicsItem
{
public:
	/**
	 * Constructor
	 * Sets the pointer to connected nodes
	 * @param sourceNode : node where the weight come from (usually the bottom layer)
	 * @param destNode : node where the weight goes to (usually the top layer)
	 */
	Edge(Node *sourceNode, Node *destNode);
	/**
	 * This function update the node classes connected to this edge object.
	 * The Edge object must be created before calling it (cannot be called in the constructor)
	 */
	void init();
	/**
	 * Adjust the visual edge to fit with the geometry of the nodes
	 */
	void adjust();
	/**
	 * Function called to change state and the order priority of the edge object when
	 * a connected node is selected
	 * @param isSelected : true if a connected node is selected
	 */
	void changeOnSelected(bool isSelected);
	/**
	 * This function is called to change the state of the edge object when a connected node is locked, so the color of the edge is kept
	 */
	void lockColor(bool);
	/**
	 * Setter function, weightValue of the ART network
	 */
	void setWeightValue(double value){weightValue = value;}
	/**
	 * Getter function, weightValue of the ART network
	 */
	double getWeightValue(){return weightValue;}

	const Node* getDest() const {
		return dest;
	}

	const Node* getSource() const {
		return source;
	}

	void forceDisplayDescription(bool);

	void setlayer(int idLayer){this->layer_ = idLayer;}

protected:
	/**
	 * This function defines the bounding rect around the graphic item
	 * @return QRectF object
	 */
	QRectF boundingRect() const;
	/**
	 * the paint function, draws all the Graphic items
	 */
	void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

private:

	Node *source, *dest;
	QPointF sourcePoint, destPoint;
	qreal arrowSize;

	bool m_isSelected;
	bool m_isLockedColor;
	double weightValue;

	int id;
	int layer_;

};

#endif
