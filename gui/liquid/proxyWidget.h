/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __LIQUID_PROXY_WIDGET_H__
#define __LIQUID_PROXY_WIDGET_H__

#include <QGraphicsProxyWidget>
#include <QPainter>
#include <QBrush>
#include <QStyleOptionGraphicsItem>
#include <QDebug>


// SIZING HACK: clean this up. please.
#include <QTreeView>

//#include "lod.h"

namespace liquid {

// Generic LOD wrapper
template<typename ItemT>
class LODItem: public ItemT{
    // error if we don't inherit from LODCapable
    //typedef _t ItemT::must_inherit_LODCapable;

public:
    LODItem(QGraphicsItem* parent=0)
        : ItemT(parent){
        // this seems to mostly hide the effects of LOD stuff... (and gives
        // a *significant* performance improvement)
        //ItemT::setCacheMode(QGraphicsItem::ItemCoordinateCache);
        // ItemCoordinateCache is definitely faster
        ItemT::setCacheMode(QGraphicsItem::DeviceCoordinateCache);
    }

    // Ideas for improving performance at mid-range LODs:
    //  -> Disable text antialiasing
    //  -> ??
    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget){
        const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
        if(lod < 0.5){
            if(lod < 0.25){
                // one of the few times that an explicit call through
                // this-> is ever necessary:
                painter->fillRect(this->boundingRect(), option->palette.color(QPalette::Window));
            }else{
                painter->setBrush(QBrush(option->palette.color(QPalette::Window)));
                painter->setPen(Qt::NoPen);
                painter->drawPath(this->shape());
            }
            return;
        }else{
            ItemT::paint(painter, option, widget);
        }
    }
};


// Add LOD support to the standard QGraphicsProxyWidget
class ProxyWidget: public QGraphicsProxyWidget{
    Q_OBJECT
public:
    ProxyWidget(QGraphicsItem* parent=0, Qt::WindowFlags wFlags=0)
        : QGraphicsProxyWidget(parent, wFlags){

        // NB: with ItemCoordinateCache, LOD stuff is ineffective (since painting is always done at 100%)
        //setCacheMode(QGraphicsItem::DeviceCoordinateCache);
        setCacheMode(QGraphicsItem::ItemCoordinateCache);

        #ifdef QT_PROFILE_GRAPHICSSCENE
            setProfileName("liquid::ProxyWidget");
        #endif // def QT_PROFILE_GRAPHICSSCENE
    }

    virtual void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget){

        const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
        if(lod < 0.5){
            if(lod < 0.25){
                painter->fillRect(boundingRect(), option->palette.color(QPalette::Window));
            }else{
                painter->setBrush(QBrush(option->palette.color(QPalette::Window)));
                painter->setPen(Qt::NoPen);
                painter->drawPath(shape());
            }
            return;
        }else{
            QGraphicsProxyWidget::paint(painter, option, widget);
        }

        #ifdef CAUV_DEBUG_DRAW_LAYOUT
        painter->setPen(QPen(QColor(120,10,100,64)));
        painter->setBrush(Qt::NoBrush);
        painter->drawRect(boundingRect());
        #endif // def CAUV_DEBUG_DRAW_LAYOUT
    }
    /*
    virtual void mousePressEvent(QGraphicsSceneMouseEvent *event){
        QGraphicsProxyWidget::mousePressEvent(event);

        qDebug() << "size"  << size();
        qDebug() << "minimumSize"  << minimumSize();
        qDebug() << "maximumSize"  << maximumSize();
        qDebug() << "sizeHint(Qt::PreferredSize)"  << sizeHint(Qt::PreferredSize);
        qDebug() << "sizeHint(Qt::MinimumSize)"  << sizeHint(Qt::MinimumSize);
        qDebug() << "sizeHint(Qt::MaximumSize)"  << sizeHint(Qt::MaximumSize);
        qDebug() << "effectiveSizeHint(Qt::PreferredSize)"  << effectiveSizeHint(Qt::PreferredSize);
        qDebug() << "effectiveSizeHint(Qt::MinimumSize)"  << effectiveSizeHint(Qt::MinimumSize);
        qDebug() << "effectiveSizeHint(Qt::MaximumSize)"  << effectiveSizeHint(Qt::MaximumSize);
    }*/

    virtual void setWidget(QWidget * widget){
        if(QTreeView * view = dynamic_cast<QTreeView*>(widget)) {
            connect(view, SIGNAL(expanded(QModelIndex)), this, SLOT(updateGeometry()));
            connect(view, SIGNAL(collapsed(QModelIndex)), this, SLOT(updateGeometry()));
        }
        QGraphicsProxyWidget::setWidget(widget);
        this->setMinimumSize(0, 0);
    }

    // for completely different reasons make sure that geometry() is an
    // integral size: (so that in layouts item positions stay at integers,
    // and lines and text all draw nicely)
    virtual void setGeometry(const QRectF & rect){
        QGraphicsProxyWidget::setGeometry(QRectF(rect.toRect()));
    }

public Q_SLOTS:
    virtual void updateGeometry(){
        // https://bugreports.qt-project.org/browse/QTBUG-14622
        QGraphicsProxyWidget::updateGeometry();
        QGraphicsLayoutItem::updateGeometry();
    }
};

} // namespace liquid

#endif // ndef __LIQUID_PROXY_WIDGET_H__
