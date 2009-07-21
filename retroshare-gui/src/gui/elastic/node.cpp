/****************************************************************************
**
** Copyright (C) 2006-2007 Trolltech ASA. All rights reserved.
**
** This file is part of the example classes of the Qt Toolkit.
**
** This file may be used under the terms of the GNU General Public
** License version 2.0 as published by the Free Software Foundation
** and appearing in the file LICENSE.GPL included in the packaging of
** this file.  Please review the following information to ensure GNU
** General Public Licensing requirements will be met:
** http://trolltech.com/products/qt/licenses/licensing/opensource/
**
** If you are unsure which license is appropriate for your use, please
** review the following information:
** http://trolltech.com/products/qt/licenses/licensing/licensingoverview
** or contact the sales department at sales@trolltech.com.
**
** In addition, as a special exception, Trolltech gives you certain
** additional rights. These rights are described in the Trolltech GPL
** Exception version 1.0, which can be found at
** http://www.trolltech.com/products/qt/gplexception/ and in the file
** GPL_EXCEPTION.txt in this package.
**
** In addition, as a special exception, Trolltech, as the sole copyright
** holder for Qt Designer, grants users of the Qt/Eclipse Integration
** plug-in the right for the Qt/Eclipse Integration to link to
** functionality provided by Qt Designer and its related libraries.
**
** Trolltech reserves all rights not expressly granted herein.
**
** This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
** WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
**
****************************************************************************/

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QPainter>
#include <QStyleOption>
#include <QMenu>

#include "edge.h"
#include "arrow.h"
#include "node.h"
#include "graphwidget.h"
#include <math.h>
#include "rsiface/rspeers.h"

Node::Node(GraphWidget *graphWidget, uint32_t t, std::string id_in, std::string n)
    : graph(graphWidget), ntype(t), id(id_in), name(n),
      mDeterminedBB(false)
{
    setFlag(ItemIsMovable);
    setZValue(1);
}

void Node::addEdge(Edge *edge)
{
    edgeList << edge;
    edge->adjust();
}

QList<Edge *> Node::edges() const
{
    return edgeList;
}

void Node::addArrow(Arrow *arrow)
{
    arrowList << arrow;
    arrow->adjust();
}

QList<Arrow *> Node::arrows() const
{
    return arrowList;
}

void Node::calculateForces()
{
    if (!scene() || scene()->mouseGrabberItem() == this) {
        newPos = pos();
        return;
    }
    
    // Sum up all forces pushing this item away
    qreal xvel = 0;
    qreal yvel = 0;
    foreach (QGraphicsItem *item, scene()->items()) {
        Node *node = qgraphicsitem_cast<Node *>(item);
        if (!node)
            continue;

        QLineF line(mapFromItem(node, 0, 0), QPointF(0, 0));
        qreal dx = line.dx();
        qreal dy = line.dy();
        double l = 2.0 * (dx * dx + dy * dy);
        if (l > 0) {
            xvel += (dx * 150.0) / l;
            yvel += (dy * 150.0) / l;
        }
    }


    // Now subtract all forces pulling items together
    double weight = sqrt(edgeList.size() + 1) * 10;
    foreach (Edge *edge, edgeList) {
        QPointF pos;
        if (edge->sourceNode() == this)
            pos = mapFromItem(edge->destNode(), 0, 0);
        else
            pos = mapFromItem(edge->sourceNode(), 0, 0);
        xvel += pos.x() / weight;
        yvel += pos.y() / weight;
    }


    // Now subtract all forces pulling items together
    // alternative weight??
    weight = sqrt(arrowList.size() + 1) * 10;
    foreach (Arrow *arrow, arrowList) {
        QPointF pos;
        if (arrow->sourceNode() == this)
            pos = mapFromItem(arrow->destNode(), 0, 0);
        else
            pos = mapFromItem(arrow->sourceNode(), 0, 0);
        xvel += pos.x() / weight;
        yvel += pos.y() / weight;
    }

    // push away from edges too.
    QRectF sceneRect = scene()->sceneRect();
    int mid_x = (sceneRect.left() + sceneRect.right()) / 2;
    int mid_y = (sceneRect.top() + sceneRect.bottom()) / 2;

    if (qAbs(xvel) < 0.1 && qAbs(yvel) < 0.1)
        xvel = yvel = 0;

    newPos = pos() + QPointF(xvel, yvel);
    newPos.setX(qMin(qMax(newPos.x(), sceneRect.left() + 10), sceneRect.right() - 10));
    newPos.setY(qMin(qMax(newPos.y(), sceneRect.top() + 10), sceneRect.bottom() - 10));

    if (ntype == ELASTIC_NODE_TYPE_OWN)
    {
	/* own one always goes in the middle */
	newPos.setX(mid_x);
	newPos.setY(mid_y);
    }
}

bool Node::advance()
{
    if (newPos == pos())
        return false;

    setPos(newPos);
    return true;
}

QRectF Node::boundingRect() const
{
    qreal adjust = 2;
    /* add in the size of the text */
    qreal realwidth = 40;
    if (mDeterminedBB)
    {
	realwidth = mBBWidth + adjust;
    }
    if (realwidth < 23 + adjust)
    {
    	realwidth = 23 + adjust;
    }

    return QRectF(-10 - adjust, -10 - adjust,
                  realwidth, 23 + adjust);
                 // 23 + adjust, 23 + adjust);
}


//QPainterPath Node::shape() const
//{
//    QPainterPath path;
//    path.addEllipse(-10, -10, 20, 20);
//    return path;
//}

void Node::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *)
{
    painter->setPen(Qt::NoPen);
    painter->setBrush(Qt::darkGray);
    painter->drawEllipse(-7, -7, 20, 20);

    QColor col0, col1;
    if (ntype == ELASTIC_NODE_TYPE_OWN)
    {
    	col0 = QColor(Qt::yellow);
    	col1 = QColor(Qt::darkYellow);
    }
    else if (ntype == ELASTIC_NODE_TYPE_FRIEND)
    {
    	col0 = QColor(Qt::green);
    	col1 = QColor(Qt::darkGreen);
    }
    else if (ntype == ELASTIC_NODE_TYPE_AUTHED)
    {
    	//col0 = QColor(Qt::cyan);
    	//col1 = QColor(Qt::darkCyan);
    	//col0 = QColor(Qt::blue);

    	col0 = QColor(Qt::cyan);
    	col1 = QColor(Qt::darkBlue);
    }
    else if (ntype == ELASTIC_NODE_TYPE_MARGINALAUTH)
    {
    	col0 = QColor(Qt::magenta);
    	col1 = QColor(Qt::darkMagenta);
    }
    else
    {
    	col0 = QColor(Qt::red);
    	col1 = QColor(Qt::darkRed);
    }

    QRadialGradient gradient(-3, -3, 10);
    if (option->state & QStyle::State_Sunken) {
        gradient.setCenter(3, 3);
        gradient.setFocalPoint(3, 3);
        gradient.setColorAt(1, col0.light(120));
        gradient.setColorAt(0, col1.light(120));
    } else {
        gradient.setColorAt(0, col0);
        gradient.setColorAt(1, col1);
    }
    painter->setBrush(gradient);
    painter->setPen(QPen(Qt::black, 0));
    painter->drawEllipse(-10, -10, 20, 20);
    painter->drawText(-10, 0, QString::fromStdString(name));

    if (!mDeterminedBB)
    {
    	QRect textBox = painter->boundingRect(-10, 0, 400, 20, 0, QString::fromStdString(name));
	mBBWidth = textBox.width();
	mDeterminedBB = true;
    }
}

QVariant Node::itemChange(GraphicsItemChange change, const QVariant &value)
{
    switch (change) {
    case ItemPositionHasChanged:
        foreach (Edge *edge, edgeList)
            edge->adjust();
        foreach (Arrow *arrow, arrowList)
            arrow->adjust();
        graph->itemMoved();
        break;
    default:
        break;
    };

    return QGraphicsItem::itemChange(change, value);
}

void Node::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    update();
    QGraphicsItem::mousePressEvent(event);
}

void Node::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    update();
    QGraphicsItem::mouseReleaseEvent(event);
}

void Node::contextMenuEvent(QGraphicsSceneContextMenuEvent *event)
{
	RsPeerDetails details;
	if (!rsPeers->getPeerDetails(id, details))
	{
		event->accept();
		return;
	}

	/* no events for self */	
	if (ntype == ELASTIC_NODE_TYPE_OWN) 
	{
		event->accept();
		return;
	}

	QString menuTitle = "Menu for ";
	menuTitle += QString::fromStdString(details.name);

	QMenu menu;
     	QAction *titleAction = menu.addAction(menuTitle);
	titleAction->setEnabled(false);

     switch(ntype)
	{
		case ELASTIC_NODE_TYPE_OWN:
		{
			break;
		}
		case ELASTIC_NODE_TYPE_FRIEND:
		{

     			//QAction *rmAction = menu.addAction("Remove Friend");
     			//QAction *chatAction = menu.addAction("Chat");
     			//QAction *msgAction = menu.addAction("Msg");
			break;
		}
		case ELASTIC_NODE_TYPE_AUTHED:
		{
     			//QAction *addAction = menu.addAction("Add Friend");
			break;
		}
		case ELASTIC_NODE_TYPE_MARGINALAUTH:
		{
     			//QAction *makeAction = menu.addAction("Make Friend");
			break;
		}
		default:
		case ELASTIC_NODE_TYPE_FOF:
		{
     			//QAction *makeAction = menu.addAction("Make Friend");
			break;
		}
	}
     QAction *selectedAction = menu.exec(event->screenPos());
} 

