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

#ifndef __CAUV_GUINODEMANAER_H__
#define __CAUV_GUINODEMANAER_H__

#include <debug/cauv_debug.h>
#include <gui/core/model/node.h>
#include <QGraphicsScene>
#include <liquid/node.h>
#include <liquid/layout.h>
#include <map>

namespace cauv {
namespace gui {


class ManagedNode {
public:
    template<class T, class S>
    static T * getLiquidNodeFor(boost::shared_ptr<S> node, bool relayout = true){
        T * liquidNode;
        liquid::LiquidNode * ln = m_liquidNodes[node];
        if(!ln){
            if(!m_scene){
                error() << "ManagedNode must have a scene set before calling getLiquidNodeFor(...)";
            }
            debug() << "getLiquidNodeFor" << node->nodePath() << "creating new node";
            liquidNode = new T(node);
            m_scene->addItem(liquidNode);
        } else {
            debug() << "getLiquidNodeFor" << node->nodePath() << "returning existing node";
            liquidNode = static_cast<T*>(ln);
        }

        if(relayout)
            liquid::LayoutItems::updateLayout(m_scene);

        return liquidNode;
    }
    
    // !!! FIXME: this needs to be set per-model, otherwise nested scenes break!
    static void setScene(QGraphicsScene * scene);

protected:
    ManagedNode(liquid::LiquidNode* ln, boost::shared_ptr<Node> node);
    static void unRegisterNode(liquid::LiquidNode *ln);

private:
    static void registerNode(liquid::LiquidNode* ln, boost::shared_ptr<Node> node);
    typedef std::map<boost::shared_ptr<Node>, liquid::LiquidNode*> t_map;
    static t_map m_liquidNodes;
    static QGraphicsScene * m_scene;
};


} // namespace gui
} // namespace cauv


#endif // __CAUV_GUINODEMANAER_H__
