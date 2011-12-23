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

#include "layout.h"

#include <cmath>
#include <cstdlib>

#include <QGraphicsItem>
#include <QGraphicsScene>

#include "arcSource.h"
#include "arcSink.h"
#include "arc.h"


// - static helper stuff
// TODO: second derivative & NR optimisation
struct EnergyChange{
    QPointF  d_dl;
    //QMatrix2x2 d2_dx2;
    EnergyChange& operator+=(EnergyChange const& r){ d_dl += r.d_dl; return *this; }
    EnergyChange& operator-=(EnergyChange const& r){ d_dl -= r.d_dl; return *this; }
};

static float length(QPointF l){
    return std::sqrt(l.x()*l.x()+l.y()*l.y());
}


//        E = 0.005 * (4e3/l - 0.5*l^2)
//    dE/dl = 0.005 * (-4e3/l^2 - l)
// d2E/dl^2 = 0.005 * (8e3/l^3)
static EnergyChange dEdXForLink(QPointF link){
    float l = length(link);
    if(l <  1) l =  1;
    EnergyChange r = {
        QPointF(-0.005 * (link - 4e3*link / (l*l))),
        //QMatrix2x2()
    };
    return r;
}

// - LayoutItems Implementation

std::set<liquid::AbstractArcSink*> liquid::LayoutItems::g_sinks;
std::set<liquid::AbstractArcSource*> liquid::LayoutItems::g_srcs;
std::set<liquid::Arc*> liquid::LayoutItems::g_arcs;

/*void liquid::LayoutItems::updateLayout(QGraphicsScene* scene){
    // TODO: probably want some locking here
    std::map<AbstractArcSink*, QGraphicsItem*> sink_parent_lut;
    std::map<AbstractArcSource*, QGraphicsItem*> src_parent_lut;
    for(std::set<AbstractArcSink*>::const_iterator i = g_sinks.begin(); i != g_sinks.end(); i++)
        if((*i)->scene() == scene)
            sink_parent_lut[*i] = (*i)->ultimateParent();
    for(std::set<AbstractArcSource*>::const_iterator i = g_srcs.begin(); i != g_srcs.end(); i++)
        if((*i)->scene() == scene)
            src_parent_lut[*i] = (*i)->ultimateParent();
    
    const int Num_Iters = 10;
    float lambda = 1.0;    
    for(int iters = 0; iters < Num_Iters; iters++, lambda *= 0.8){
        typedef std::map<QGraphicsItem*, EnergyChange> qgi_ec_map;
        qgi_ec_map item_energy_change;
        for(std::set<Arc*>::const_iterator i = g_arcs.begin(); i != g_arcs.end(); i++)
            if((*i)->scene() == scene){
                AbstractArcSource* const arc_src = (*i)->source();
                QGraphicsItem* const src_parent = src_parent_lut[arc_src];
                const QPointF src_pos = arc_src->scenePos();
                foreach(AbstractArcSink* arc_sink, (*i)->sinks()){ 
                    QGraphicsItem* const sink_parent = sink_parent_lut[arc_sink];
                    const QPointF sink_pos = arc_sink->scenePos();
                    EnergyChange de = dEdXForLink(sink_pos - src_pos);
                    item_energy_change[src_parent] -= de;
                    item_energy_change[sink_parent] += de;
                }
            }
        qgi_ec_map::iterator j;
        for(j = item_energy_change.begin(); j != item_energy_change.end(); j++)
            j->first->setPos(j->first->pos() + lambda * j->second.d_dl);
    }
}*/

static QPointF consensus(QList<QPointF> l){
    /*const int Iters = 1;
    QPointF best;
    int best_support = 0;
    float support_mhdist = 100000;
    for(int it = 0; it < Iters; it++){
        QList<QPointF> sample;
        for(int i = 0; i < (l.size()+1)/2 && i < 4; i++)
            sample << l[std::rand()%l.size()];
        QPointF sac(0,0);
        foreach(QPointF p, sample)
            sac += p;
        sac /= sample.size();
        int support = 0;
        foreach(QPointF p, l)
            if(std::fabs((p - sac).manhattanLength()) < support_mhdist)
                support++;
        if(support >= best_support){
            best_support = support;
            best = sac;
        }
    }
    return best;*/
    QPointF r(0,0);
    foreach(QPointF p, l)
        r += p;
    return r / l.size();
}

void liquid::LayoutItems::updateLayout(QGraphicsScene* scene){
    // TODO: probably want some locking here
    std::map<AbstractArcSink*, QGraphicsItem*> sink_parent_lut;
    std::map<AbstractArcSource*, QGraphicsItem*> src_parent_lut;
    for(std::set<AbstractArcSink*>::const_iterator i = g_sinks.begin(); i != g_sinks.end(); i++)
        if((*i)->scene() == scene)
            sink_parent_lut[*i] = (*i)->ultimateParent();
    for(std::set<AbstractArcSource*>::const_iterator i = g_srcs.begin(); i != g_srcs.end(); i++)
        if((*i)->scene() == scene)
            src_parent_lut[*i] = (*i)->ultimateParent();
    
    const int Num_Iters = 30;
    float lambda = 1.0;
    typedef std::map<QGraphicsItem*, float> qgi_f_map;
    qgi_f_map base_offset_for_item;
    for(int iters = 0; iters < Num_Iters; iters++, lambda *= 0.8){
        typedef std::map<QGraphicsItem*, QList<QPointF> > qgi_position_vote_map;
        qgi_position_vote_map item_position_votes;
        qgi_f_map last_offset_for_item;
        for(std::set<Arc*>::const_iterator i = g_arcs.begin(); i != g_arcs.end(); i++)
            if((*i)->scene() == scene){
                AbstractArcSource* const arc_src = (*i)->source();
                QGraphicsItem* const src_parent = src_parent_lut[arc_src];
                const QPointF src_pos = arc_src->scenePos();
                qgi_f_map::iterator f = last_offset_for_item.find(src_parent);
                if(f == last_offset_for_item.end()){
                    f = base_offset_for_item.find(src_parent);
                    if(f == base_offset_for_item.end())
                        f = base_offset_for_item.insert(qgi_f_map::value_type(src_parent, 0)).first;
                }
                QPointF offset(80, f->second);
                foreach(AbstractArcSink* arc_sink, (*i)->sinks()){
                    QGraphicsItem* const sink_parent = sink_parent_lut[arc_sink];
                    const QPointF sink_pos = arc_sink->scenePos();
                    const QPointF vote = src_pos + offset + sink_parent->scenePos() - sink_pos;
                    item_position_votes[sink_parent] << vote;
                    offset += QPointF(0,20+sink_parent->boundingRect().height());
                }
                last_offset_for_item[src_parent] = offset.y();
                base_offset_for_item[src_parent] = 40 - (offset.y() - base_offset_for_item[src_parent]) / 2;
            }
        qgi_position_vote_map::iterator j;
        for(j = item_position_votes.begin(); j != item_position_votes.end(); j++){
            if(j->second.size())
                j->first->setPos(consensus(j->second));
        }
    }
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

