/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "nodePathFilter.h"

#include <boost/shared_ptr.hpp>

#include <debug/cauv_debug.h>

#include "model/node.h"

using namespace cauv;
using namespace cauv::gui;

NodePathFilter::NodePathFilter(QObject * parent) : QObject(parent), m_text(""){
}

void NodePathFilter::setText(QString const& string){
    m_text = string;
    Q_EMIT filterChanged();
}

QString NodePathFilter::getText(){
    return m_text;
}

bool NodePathFilter::containsText(boost::shared_ptr<Node> const& node){
    CAUV_LOG_DEBUG(7, "NODE = " << node);
    CAUV_LOG_DEBUG(7, "PATH = " << node->nodePath());
    CAUV_LOG_DEBUG(7, "TEXT = " << getText().toStdString());

    if(QString::fromStdString(node->nodePath()).contains(getText(), Qt::CaseInsensitive))
        return true;

    // might match somewhere in a child nodes path
    foreach(boost::shared_ptr<Node> const& child, node->getChildren()){
        if(containsText(child)) return true;
    }

    return false;
}

bool NodePathFilter::filter(boost::shared_ptr<Node> const& node){
    return getText().isEmpty() || containsText(node);
}

