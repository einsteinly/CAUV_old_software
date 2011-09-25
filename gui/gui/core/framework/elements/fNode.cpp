#include "fNode.h"

#include <set>

#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>
#include <utility/qt_streamops.h>

#include <liquid/button.h>
#include <liquid/nodeHeader.h>
#include <liquid/connectionSink.h>
#include <liquid/arcSink.h>

#include "style.h"
#include "nodeInput.h"

using cauv::gui::FNode;
using namespace liquid;

class TestLayoutItem: public QGraphicsLayoutItem,
                      public QGraphicsPathItem{
    public:
        TestLayoutItem(QRectF preferred_geom)
            : QGraphicsLayoutItem(),
              QGraphicsPathItem(),
              m_preferred_geom(preferred_geom){
            setSizePolicy(QSizePolicy::Fixed);
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
    addItem(new TestLayoutItem(QRectF(0,-5,50,10)));
    addItem(new TestLayoutItem(QRectF(0,-5,20,10)));
    
    //addItem(new NodeInput(m_style, NodeIOType::Image, true, this));
    TempConnectionSink *s = new TempConnectionSink();
    addItem(new liquid::ArcSink(Image_Arc_Style, Required_Image_Input, s));

    addItem(new TestLayoutItem(QRectF(0,-5,90,10)));
    //addItem(new TestLayoutItem(QRectF(0,-5,120,10)));
    //addItem(new TestLayoutItem(QRectF(0,-5,100,10)));
    //addItem(new TestLayoutItem(QRectF(0,-5,80,10)));
    
    addItem(new liquid::ArcSink(Image_Arc_Style, Required_Image_Input, s));
    addItem(new liquid::ArcSink(Image_Arc_Style, Required_Image_Input, s));
    addItem(new liquid::ArcSink(Image_Arc_Style, Required_Image_Input, s));

    //addItem(new NodeInput(m_style, NodeIOType::Image, false, this));
    //addItem(new NodeInput(m_style, NodeIOType::Parameter, true, this));
    //addItem(new NodeInput(m_style, NodeIOType::Parameter, false, this));
    
    addItem(new TestLayoutItem(QRectF(0,-5,80,10)));
}

