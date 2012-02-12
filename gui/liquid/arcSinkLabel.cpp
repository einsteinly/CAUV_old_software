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

#include <debug/cauv_debug.h>

using namespace liquid;


// - ArcSinkLabel
ArcSinkLabel::ArcSinkLabel(liquid::ArcStyle const& of_style,
                           liquid::CutoutStyle const& with_cutout,
                           liquid::LiquidNode* node,
                           liquid::ConnectionSink * sink,
                           std::string const& id)
    : QGraphicsWidget(node),
      RequiresCutout(),
      m_arc_sink(new liquid::ArcSink(of_style, with_cutout, sink)),
      m_text(NULL){

    QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(
        Qt::Horizontal, this
    );
    hlayout->setSpacing(0);
    hlayout->setContentsMargins(0,0,0,0);

    m_arc_sink->setParentItem(this);
    hlayout->addItem(m_arc_sink);
    hlayout->setAlignment(m_arc_sink, Qt::AlignVCenter | Qt::AlignLeft);

    QLabel* text_label = new QLabel(QString::fromStdString(id));
    text_label->setTextInteractionFlags(Qt::NoTextInteraction);
    text_label->setFont(node->style().text.font);

    QPalette transparent_bg = palette();
    for(int i=0; i < QPalette::NColorGroups; i++){
         QColor color = transparent_bg.brush(QPalette::ColorGroup(i), QPalette::Window).color();
         color.setAlpha(0);
         transparent_bg.setBrush(QPalette::ColorGroup(i), QPalette::Window, QBrush(color));
    }
    text_label->setPalette(transparent_bg);

    m_text = new QGraphicsProxyWidget();
    m_text->setWidget(text_label);
    hlayout->addItem(m_text);
    hlayout->setAlignment(m_text, Qt::AlignVCenter | Qt::AlignLeft);

    hlayout->setItemSpacing(1, 4.0);

    hlayout->addStretch(1);

    setLayout(hlayout);

    connect(node, SIGNAL(xChanged()), m_arc_sink, SIGNAL(geometryChanged()));
    connect(node, SIGNAL(yChanged()), m_arc_sink, SIGNAL(geometryChanged()));
}

ArcSinkLabel::~ArcSinkLabel(){
}

liquid::AbstractArcSink* ArcSinkLabel::sink() const{
    return m_arc_sink;
}

QList<liquid::CutoutStyle> ArcSinkLabel::cutoutGeometry() const{
    QList<liquid::CutoutStyle> sink_cutouts =  m_arc_sink->cutoutGeometry();
    QList<liquid::CutoutStyle> r;
    foreach(liquid::CutoutStyle const& c, sink_cutouts){
        liquid::CutoutStyle t = c;
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
    painter->setPen(QPen(QColor(20,180,40,64)));
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(boundingRect());
    debug(11) << "arc sink is at:"
              << m_arc_sink->pos().x()
              << m_arc_sink->pos().y()
              << m_arc_sink->geometry().width()
              << m_arc_sink->geometry().height()
              << m_arc_sink->parentItem();
}
