/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#include "layout.h"

#include <cmath>
#include <cstdlib>

#include <QGraphicsItem>
#include <QGraphicsScene>

#include <QEasingCurve>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QObject>

#include <utility/graphviz.h>
#include <utility/bimap.h>
#include <utility/string.h>

#include <debug/cauv_debug.h>

#include "arcSource.h"
#include "arcSink.h"
#include "arc.h"

// - LayoutItems Implementation

std::set<liquid::AbstractArcSink*> liquid::LayoutItems::g_sinks;
std::set<liquid::AbstractArcSource*> liquid::LayoutItems::g_srcs;
std::set<liquid::Arc*> liquid::LayoutItems::g_arcs;


void liquid::LayoutItems::updateLayout(QGraphicsScene* scene){
    namespace gv = graphviz;

    std::map<AbstractArcSink*, QGraphicsItem*> sink_parent_lut;
    std::map<AbstractArcSource*, QGraphicsItem*> src_parent_lut;
    std::set<QGraphicsItem*> nodes;
    cauv::bimap<std::string, QGraphicsItem*> node_names;
    int next_name = 0;

    for(std::set<AbstractArcSink*>::const_iterator i = g_sinks.begin(); i != g_sinks.end(); i++)
        if((*i)->scene() == scene){
            QGraphicsItem* node = (*i)->ultimateParent();
            sink_parent_lut[*i] = node;
            if(!nodes.count(*i)){
                node_names.insert(node, mkStr() << next_name++);
                nodes.insert(node);
            }
        }
    for(std::set<AbstractArcSource*>::const_iterator i = g_srcs.begin(); i != g_srcs.end(); i++)
        if((*i)->scene() == scene){
            QGraphicsItem* node = (*i)->ultimateParent();        
            src_parent_lut[*i] = node;
            if(!nodes.count(*i)){
                node_names.insert(node, mkStr() << next_name++);
                nodes.insert(node);
            }
        }
    
    gv::Context c;

    float dpi = 96;

    gv::Graph g("nodes", AGDIGRAPH);
    g.addGraphAttr("dpi", dpi);
    g.addGraphAttr("rankdir", "LR");
    g.addGraphAttr("nodesep", 1);
    g.addGraphAttr("ranksep", 1);
    g.addNodeAttr("shape","box");
    g.addNodeAttr("height", 1);
    g.addNodeAttr("width", 1);

    foreach(QGraphicsItem* n, nodes)
    {
        gv::Node gn = g.node(node_names[n]);
        gn.attr("width", n->boundingRect().width() / dpi);
        gn.attr("height", n->boundingRect().height() / dpi);
    }
    foreach(Arc* a, g_arcs)
    {   
        QGraphicsItem* from_node = src_parent_lut[a->source()];
        gv::Node gfrom_node = g.node(node_names[from_node]);
        
        foreach(AbstractArcSink* s, a->sinks()){
            QGraphicsItem* to_node = sink_parent_lut[s];
            gv::Node gto_node = g.node(node_names[to_node]);

            g.edge(gfrom_node, gto_node);
        }
    }

    gv::gvLayout(c.get(), g.get(), "dot");
    gv::gvRenderFilename(c.get(), g.get(), "png", "out.png");
    
    QRectF layout_rect(0,0,0,0);
    foreach(const gv::Node& gn, g.nodes)
    {
        QGraphicsItem* n = node_names[gn.name()];
        QPointF pos = QPointF(gn.coord().x, gn.coord().y) - n->boundingRect().center();
        layout_rect |= n->boundingRect().translated(pos);
    }

    QParallelAnimationGroup *anim_group = new QParallelAnimationGroup;
    const int duration = 2500;

    foreach(const gv::Node& gn, g.nodes)
    {
        QGraphicsItem* n = node_names[gn.name()];
        QPointF centre_pos = QPointF(gn.coord().x, gn.coord().y) - n->boundingRect().center();
        QPointF end_pos = (centre_pos - layout_rect.center()).toPoint();

        QGraphicsObject* as_qgo = dynamic_cast<QGraphicsObject*>(n);
        if(as_qgo){
            QPropertyAnimation *anim = new QPropertyAnimation(as_qgo, "pos");
            anim->setDuration(duration);
            anim->setStartValue(n->pos());
            anim->setEndValue(end_pos);
            anim->setEasingCurve(QEasingCurve::InOutQuad);
            anim_group->addAnimation(anim);
        }else{
            n->setPos(end_pos);
        }
    }

    anim_group->start();
 
    gv::gvFreeLayout(c.get(), g.get());
}

liquid::LayoutItems::LayoutItems(AbstractArcSink* sink){
    registerSinkItem(sink);
}

liquid::LayoutItems::LayoutItems(AbstractArcSource* src){
    registerSourceItem(src);
}

liquid::LayoutItems::LayoutItems(Arc* arc){
    registerConnection(arc);
}

void liquid::LayoutItems::registerSinkItem(AbstractArcSink* sink){
    g_sinks.insert(sink);
}

void liquid::LayoutItems::registerSourceItem(AbstractArcSource* source){
    g_srcs.insert(source);
}

void liquid::LayoutItems::registerConnection(Arc* arc){
    g_arcs.insert(arc);
}

void liquid::LayoutItems::unRegisterSinkItem(AbstractArcSink* sink){
    if(sink)
        g_sinks.erase(sink);
}

void liquid::LayoutItems::unRegisterSourceItem(AbstractArcSource* source){
    if(source)
        g_srcs.erase(source);
}

void liquid::LayoutItems::unRegisterConnection(Arc* arc){
    if(arc)
        g_arcs.erase(arc);
}

