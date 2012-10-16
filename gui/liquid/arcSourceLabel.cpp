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

#include "arcSourceLabel.h"

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
ArcSourceLabel::ArcSourceLabel(ArcSource * arc_source,
                           LiquidNode* node,
                           QString const& id)
    : QGraphicsWidget(node),
      m_arc_source(arc_source),
      m_text(NULL),
      m_hlayout(NULL){

    /*
     *    This widget
     * o---------------------------+
     * |                           |
     * |   this->layout() (V)      |
     * | +-----------------------+ |
     * | |        h layout       | |
     * | | +---------++--------+ | |
     * | | |  label  || source | | |
     * | | +---------++--------+ | |
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

    m_hlayout = new QGraphicsLinearLayout(
        Qt::Horizontal, vlayout
    );
    m_hlayout->setSpacing(0);
    m_hlayout->setContentsMargins(0,0,0,0);
    vlayout->addItem(m_hlayout);

    LiquidLabel* text_label = new LiquidLabel(id);
    text_label->setFont(node->style().text.font);

    m_hlayout->addStretch();

    m_text = new ProxyWidget();
    m_text->setWidget(text_label);
    m_hlayout->addItem(m_text);
    m_hlayout->setAlignment(m_text, Qt::AlignVCenter | Qt::AlignRight);

    m_arc_source->setParentItem(this);
    m_hlayout->addItem(m_arc_source);
    m_hlayout->setAlignment(m_arc_source, Qt::AlignVCenter | Qt::AlignRight);

    m_hlayout->setItemSpacing(0, 4.0);
    //m_hlayout->setItemSpacing(1, 2.0);
    //m_hlayout->addStretch(1);

    setLayout(vlayout);

    connect(node, SIGNAL(xChanged()), m_arc_source, SIGNAL(geometryChanged()));
    connect(node, SIGNAL(yChanged()), m_arc_source, SIGNAL(geometryChanged()));
    
    setCacheMode(DeviceCoordinateCache);

    #ifndef CAUV_DEBUG_DRAW_LAYOUT
    setFlag(ItemHasNoContents);
    #endif // ndef CAUV_DEBUG_DRAW_LAYOUT

    #ifdef QT_PROFILE_GRAPHICSSCENE
    setProfileName("liquid::ArcSourceLabel");
    m_text->setProfileName("liquid::ArcSourceLabel::text");
    #endif // def QT_PROFILE_GRAPHICSSCENE
}

ArcSourceLabel::~ArcSourceLabel(){
}

AbstractArcSource* ArcSourceLabel::source() const{
    return m_arc_source;
}

void ArcSourceLabel::paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    Q_UNUSED(painter);
    #ifdef CAUV_DEBUG_DRAW_LAYOUT
    painter->setPen(QPen(QColor(20,180,40,64)));
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(boundingRect());
    debug(11) << "arc source is at:"
              << m_arc_source->pos().x()
              << m_arc_source->pos().y()
              << m_arc_source->geometry().width()
              << m_arc_source->geometry().height()
              << m_arc_source->parentItem();
    #endif // def CAUV_DEBUG_DRAW_LAYOUT
}

QGraphicsLinearLayout* ArcSourceLabel::hLayout() const{
    return m_hlayout;
}
