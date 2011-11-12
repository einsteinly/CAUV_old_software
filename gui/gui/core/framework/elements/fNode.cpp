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

#include "fNode.h"

#include <set>

#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>
#include <utility/qt_streamops.h>

#include <liquid/button.h>
#include <liquid/nodeHeader.h>
#include <liquid/connectionSink.h>
#include <liquid/arcSink.h>
#include <liquid/arcSource.h>
#include <liquid/arc.h>

#include "style.h"
#include "nodeInput.h"
#include "fNodeOutput.h"

using cauv::gui::FNode;
using namespace liquid;

class TestLayoutItem: public QGraphicsLayoutItem,
                      public QGraphicsPathItem{
    public:
        TestLayoutItem(QRectF preferred_geom)
            : QGraphicsLayoutItem(),
              QGraphicsPathItem(),
              m_preferred_geom(preferred_geom){
            setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
            setPen(QPen(QColor(160,20,20,64)));

            setZValue(100);
            
            QPainterPath p;
            p.addRect(m_preferred_geom.adjusted(1,1,-1,-1));
            setPath(p);
        }
        
        void setGeometry(QRectF const& rect){
            prepareGeometryChange();
            // sets geometry()
            QGraphicsLayoutItem::setGeometry(rect);
            debug() << "setGeometry" << rect << "(pos=" << pos() << ")";
            setPos(rect.topLeft() - m_preferred_geom.topLeft());

            debug() << "parent item is" << 	parentItem();
        }

        /*void updateGeometry(){
            debug() << "updateGeometry";
        }*/

    protected:
        /*QRectF boundingRect() const{
            return geometry();
        }*/

        QSizeF sizeHint(Qt::SizeHint which, QSizeF const& constraint=QSizeF()) const{
            switch(which){
                default:
                case Qt::MinimumSize:
                case Qt::PreferredSize:
                    return m_preferred_geom.size();
                case Qt::MaximumSize:
                    return m_preferred_geom.size() * 2;
            }
        }

    private:
        QRectF m_preferred_geom;
};

class TempConnectionSink: public liquid::ConnectionSink{
    public:
        virtual bool willAcceptConnection(void* from_source){
            debug() << "willAcceptConnection" << from_source;
            return true;
        }

        virtual ConnectionStatus doAcceptConnection(void* from_source){
            debug() << "doAcceptConnection" << from_source;
            return Rejected;
        }
};

class MyConnectionSource{
    public:
        void moo(){
            debug() << "MyConnectionSource::moo";
        }
};


FNode::FNode(Manager& m, QGraphicsItem *parent)
    : liquid::LiquidNode(F_Node_Style, parent),
      ManagedElement(m){

    Button *collapsebutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/collapse_button"), NULL, this
    );
    m_header->addButton("collapse", collapsebutton);

    Button *execbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"), NULL, this
    );
    m_header->addButton("exec", execbutton);
    
    Button *dupbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/dup_button"), NULL, this
    );
    m_header->addButton("duplicate", dupbutton);
    

    setSize(QSizeF(104,130));
    
    // !!!
    
    TempConnectionSink *k = new TempConnectionSink();
    MyConnectionSource *c = new MyConnectionSource();

    addItem(new liquid::ArcSink(Image_Arc_Style, Required_Image_Input, k));
    addItem(new liquid::ArcSink(Image_Arc_Style, Optional_Image_Input, k));
    //addItem(new liquid::ArcSink(Param_Arc_Style, Required_Param_Input, k));
    addItem(new liquid::ArcSink(Param_Arc_Style, Optional_Param_Input, k));

    addItem(new TestLayoutItem(QRectF(0,-5,90,10)));
    //addItem(new TestLayoutItem(QRectF(0,-5,50,10)));

    //addItem(new liquid::ArcSource(c, new liquid::Arc(Image_Arc_Style)));
    //addItem(new liquid::ArcSource(c, new liquid::Arc(Image_Arc_Style)));
    //addItem(new liquid::ArcSource(c, new liquid::Arc(Param_Arc_Style)));
    addItem(new FNodeOutput(c));
    addItem(new FNodeOutput(c));
    addItem(new FNodeOutput(c));
}

