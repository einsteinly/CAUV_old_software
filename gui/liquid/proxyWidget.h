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
        }

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
    public:
        ProxyWidget(QGraphicsItem* parent=0, Qt::WindowFlags wFlags=0)
            : QGraphicsProxyWidget(parent, wFlags){
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
        }

        // for completely different reasons make sure that geometry() is an
        // integral size: (so that in layouts item positions stay at integers,
        // and lines and text all draw nicely)
        virtual void setGeometry(const QRectF & rect){
            QGraphicsProxyWidget::setGeometry(QRectF(rect.toRect()));
        }
};

} // namespace liquid

#endif // ndef __LIQUID_PROXY_WIDGET_H__
