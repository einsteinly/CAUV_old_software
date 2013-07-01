/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "arcSinkLabel.h"

#include <QGraphicsLinearLayout>
#include <QLabel>
#include <QPainter>

#include "style.h"
#include "node.h"
#include "label.h"
#include "proxyWidget.h"

#include <debug/cauv_debug.h>

using namespace liquid;


// - ArcSinkLabel
ArcSinkLabel::ArcSinkLabel(ArcSink * arc_sink,
                           LiquidNode* node,
                           QString const& id)
    : QGraphicsWidget(node),
      RequiresCutout(),
      m_arc_sink(arc_sink),
      m_text(nullptr),
      m_hlayout(nullptr){

    /*
     *    This widget
     * o---------------------------+
     * |                           |
     * |   this->layout() (V)      |
     * | +-----------------------+ |
     * | |        h layout       | |
     * | | +----++-------------+ | |
     * | | |sink||    label    | | |
     * | | +----++-------------+ | |
     * | +-----------------------+ |
     * | : other stuff added to  : |
     * | :    this->layout()     : |
     * | + - - - - - - - - - - - + |
     * | :                       : |
     * | :                       : |
     * | + - - - - - - - - - - - + |
     * |                           |
     * +---------------------------+
     */
    
    auto  vlayout = new QGraphicsLinearLayout(
        Qt::Vertical, this
    );
    vlayout->setSpacing(0);
    vlayout->setContentsMargins(0,0,0,0);    

    m_hlayout = new QGraphicsLinearLayout(
        Qt::Horizontal, vlayout
    );
    m_hlayout->setSpacing(0);
    m_hlayout->setContentsMargins(0,0,0,0);
    vlayout->addItem(m_hlayout);

    m_arc_sink->setParentItem(this);
    m_hlayout->addItem(m_arc_sink);
    m_hlayout->setAlignment(m_arc_sink, Qt::AlignVCenter | Qt::AlignLeft);

    LiquidLabel* text_label = new LiquidLabel(id);
    text_label->setFont(node->style().text.font);

    m_text = new ProxyWidget();
    m_text->setWidget(text_label);
    m_hlayout->addItem(m_text);
    m_hlayout->setAlignment(m_text, Qt::AlignVCenter | Qt::AlignLeft);

    m_hlayout->setItemSpacing(0, 4.0);
    //m_hlayout->setItemSpacing(1, 2.0);
    //m_hlayout->addStretch(1);

    setLayout(vlayout);

    connect(node, SIGNAL(xChanged()), m_arc_sink, SIGNAL(geometryChanged()));
    connect(node, SIGNAL(yChanged()), m_arc_sink, SIGNAL(geometryChanged()));
    

    setCacheMode(DeviceCoordinateCache);

    #ifndef CAUV_DEBUG_DRAW_LAYOUT
    setFlag(ItemHasNoContents);
    #endif // ndef CAUV_DEBUG_DRAW_LAYOUT 

    #ifdef QT_PROFILE_GRAPHICSSCENE
    setProfileName("liquid::ArcSinkLabel");
    m_text->setProfileName("liquid::ArkSinkLabel::text");
    #endif // def QT_PROFILE_GRAPHICSSCENE
}

ArcSinkLabel::~ArcSinkLabel(){
}

AbstractArcSink* ArcSinkLabel::sink() const{
    return m_arc_sink;
}

QList<CutoutStyle> ArcSinkLabel::cutoutGeometry() const{
    QList<CutoutStyle> sink_cutouts =  m_arc_sink->cutoutGeometry();
    QList<CutoutStyle> r;
    foreach(CutoutStyle const& c, sink_cutouts){
        CutoutStyle t = c;
        t.main_cutout.y_offset += m_arc_sink->pos().y();
        t.second_cutout.y_offset += m_arc_sink->pos().y();
        r << t;
    }
    return r;
}


void ArcSinkLabel::paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    Q_UNUSED(painter);
    #ifdef CAUV_DEBUG_DRAW_LAYOUT
    painter->setPen(QPen(QColor(20,180,40,64)));
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(boundingRect());
    debug(11) << "arc sink is at:"
              << m_arc_sink->pos().x()
              << m_arc_sink->pos().y()
              << m_arc_sink->geometry().width()
              << m_arc_sink->geometry().height()
              << m_arc_sink->parentItem();
    #endif // def CAUV_DEBUG_DRAW_LAYOUT
}

/*QGraphicsLinearLayout* ArcSinkLabel::vLayout() const{
    return dynamic_cast<QGraphicsLinearLayout*>(QGraphicsWidget::layout());
}*/

QGraphicsLinearLayout* ArcSinkLabel::hLayout() const{
    return m_hlayout;
}
