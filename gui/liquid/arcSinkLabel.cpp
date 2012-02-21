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

#include "arcSinkLabel.h"

#include <QGraphicsLinearLayout>
#include <QLabel>
#include <QPainter>

#include "style.h"
#include "node.h"
#include "label.h"

#include <debug/cauv_debug.h>

using namespace liquid;


// - ArcSinkLabel
ArcSinkLabel::ArcSinkLabel(ArcSink * arc_sink,
                           LiquidNode* node,
                           QString const& id)
    : QGraphicsWidget(node),
      RequiresCutout(),
      m_arc_sink(arc_sink),
      m_text(NULL){

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
    
    QGraphicsLinearLayout *vlayout = new QGraphicsLinearLayout(
        Qt::Vertical, this
    );
    vlayout->setSpacing(0);
    vlayout->setContentsMargins(0,0,0,0);    

    QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(
        Qt::Horizontal, vlayout
    );
    hlayout->setSpacing(0);
    hlayout->setContentsMargins(0,0,0,0);
    vlayout->addItem(hlayout);

    m_arc_sink->setParentItem(this);
    hlayout->addItem(m_arc_sink);
    hlayout->setAlignment(m_arc_sink, Qt::AlignVCenter | Qt::AlignLeft);

    LiquidLabel* text_label = new LiquidLabel(id);
    text_label->setFont(node->style().text.font);

    m_text = new QGraphicsProxyWidget();
    m_text->setWidget(text_label);
    hlayout->addItem(m_text);
    hlayout->setAlignment(m_text, Qt::AlignVCenter | Qt::AlignLeft);

    hlayout->setItemSpacing(0, 4.0);
    hlayout->setItemSpacing(1, 2.0);
    hlayout->addStretch(1);

    setLayout(vlayout);

    connect(node, SIGNAL(xChanged()), m_arc_sink, SIGNAL(geometryChanged()));
    connect(node, SIGNAL(yChanged()), m_arc_sink, SIGNAL(geometryChanged()));
    
    setCacheMode(DeviceCoordinateCache);    
    #ifndef CAUV_DEBUG_DRAW_LAYOUT
    setFlag(ItemHasNoContents);
    #endif // ndef CAUV_DEBUG_DRAW_LAYOUT
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

QGraphicsLinearLayout* ArcSinkLabel::vLayout() const{
    return dynamic_cast<QGraphicsLinearLayout*>(QGraphicsWidget::layout());
}

