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
 *     Steve Ogborne   steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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
    debug(7) << "NODE = " << node;
    debug(7) << "PATH = " << node->nodePath();
    debug(7) << "TEXT = " << getText().toStdString();

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

