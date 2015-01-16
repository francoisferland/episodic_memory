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

#include <QPainter>

#include <edge.h>
#include <node.h>

#include <math.h>

static const double Pi = 3.14159265358979323846264338327950288419717;
static double TwoPi = 2.0 * Pi;

Edge::Edge(Node *sourceNode, Node *destNode)
: arrowSize(5)
{
	m_isSelected = false;
	m_isLockedColor = false;
	weightValue = 0;
	setAcceptedMouseButtons(0);
	source = sourceNode;
	dest = destNode;
	id = -1;
	layer_ = EM_GUI::UNDEFINE;
	adjust();

}

void Edge::init()
{
	source->addEdgeDown(shared_from_this());
	dest->addEdgeUp(shared_from_this());
}

void Edge::adjust()
{
	if (!source || !dest)
		return;

	QLineF line(mapFromItem(source, 0, 0), mapFromItem(dest, 0, 0));
	qreal length = line.length();

	prepareGeometryChange();

	if (length > qreal(20.)) {
		QPointF edgeOffset((line.dx() * 10) / length, (line.dy() * 10) / length);
		sourcePoint = line.p1() + edgeOffset;
		destPoint = line.p2() - edgeOffset;
	} else {
		sourcePoint = destPoint = line.p1();
	}
}

QRectF Edge::boundingRect() const
{
	if (!source || !dest)
		return QRectF();

	qreal penWidth = 1;
	qreal extra = (penWidth + arrowSize) / 2.0;

	return QRectF(sourcePoint, QSizeF(destPoint.x() - sourcePoint.x(),
			destPoint.y() - sourcePoint.y()))
			.normalized()
			.adjusted(-extra, -extra, extra, extra);
}

void Edge::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
	if (!source || !dest)
		return;

	QPainterPath path;
	qreal leftx = source->x() < dest->x() ? source->x() : dest->x();
	qreal width = abs(source->x() - dest->x());

	if(layer_ == EM_GUI::INPUT_LAYER)
	{
		QRectF clipRect1( leftx - 10, dest->y()+50 ,width + 20 , 250 );
		QRectF clipRect2( leftx - 10, dest->y()    ,width + 20 , 30  );
		path.addRect(clipRect1);
		path.addRect(clipRect2);
	}
	else if(layer_ == EM_GUI::EVENT_LAYER)
	{
		QRectF clipRect1( leftx - 10, dest->y()+50 ,width + 20 , 138 );
		QRectF clipRect2( leftx - 10, dest->y()    ,width + 20 , 30  );
		QRectF clipRect3( leftx + 50, dest->y()+30 ,width      , 20  );
		QRectF clipRect4( leftx - 10, dest->y()+208,width + 20 , 60  );
		path.addRect(clipRect1);
		path.addRect(clipRect2);
		path.addRect(clipRect3);
		path.addRect(clipRect4);
	}

	painter->setClipping(true);
	painter->setClipPath(path);

	QLineF line(sourcePoint, destPoint);
	if (qFuzzyCompare(line.length(), qreal(0.)))
		return;

	// Draw the line itself
	if(!m_isSelected && !m_isLockedColor)
	{
		painter->setPen(QPen(Qt::black, 1, Qt::SolidLine, Qt::RoundCap, Qt::BevelJoin));
		painter->setClipping(true);
		painter->setClipPath(path);
	}
	else
	{
		painter->setPen(QPen(Qt::darkYellow, 2, Qt::SolidLine, Qt::RoundCap, Qt::BevelJoin));
		painter->setClipping(false);
	}
	painter->drawLine(line);

	// Draw the arrows
	double angle = ::acos(line.dx() / line.length());
	if (line.dy() >= 0)
		angle = TwoPi - angle;

	QPointF sourceArrowP1 = sourcePoint + QPointF(sin(angle + Pi / 3) * arrowSize,
			cos(angle + Pi / 3) * arrowSize);
	QPointF sourceArrowP2 = sourcePoint + QPointF(sin(angle + Pi - Pi / 3) * arrowSize,
			cos(angle + Pi - Pi / 3) * arrowSize);
	QPointF destArrowP1 = destPoint + QPointF(sin(angle - Pi / 3) * arrowSize,
			cos(angle - Pi / 3) * arrowSize);
	QPointF destArrowP2 = destPoint + QPointF(sin(angle - Pi + Pi / 3) * arrowSize,
			cos(angle - Pi + Pi / 3) * arrowSize);

	painter->drawPolygon(QPolygonF() << line.p1() << sourceArrowP1 << sourceArrowP2);
	painter->drawPolygon(QPolygonF() << line.p2() << destArrowP1 << destArrowP2);

	if(!m_isSelected && !m_isLockedColor)
	{
		painter->setBrush(Qt::black);
	}
	else
	{
		painter->setBrush(Qt::yellow);
		//display the weightValue
		QFont font = painter->font();
		font.setPointSize(8);
		font.setBold(true);
		painter->setFont(font);
		painter->setPen(QPen(Qt::red, 1, Qt::SolidLine, Qt::RoundCap, Qt::BevelJoin));
		painter->drawText(line.pointAt((double)(positionWeightValue+5)/10), QString::number(this->getWeightValue(),'f',2));

		//positionWeightValue%2?positionWeightValue++:positionWeightValue--;
		//positionWeightValue = positionWeightValue % 3;

	}

}

void Edge::changeOnSelected(bool isSelected)
{
	m_isSelected = isSelected;
	if(m_isSelected)
		this->setZValue(1000);
	else
		this->setZValue(6);

	update();
}

void Edge::lockColor(bool isLocked)
{
	m_isLockedColor = isLocked;
}

void Edge::forceDisplayDescription(bool isShowed)
{
	if(!source->isComplement())
	{
		source->showDescription(isShowed);
		source->hightlightNode(isShowed);
	}
	if(!dest->isComplement())
	{
		dest->showDescription(isShowed);
		dest->hightlightNode(isShowed);
	}
}
