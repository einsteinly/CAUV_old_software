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

#ifndef __LIQUID_LAYOUT_H__
#define __LIQUID_LAYOUT_H__

#include <map>
#include <set>

class QGraphicsScene;
class QGraphicsItem;

namespace liquid{

class AbstractArcSink;
class AbstractArcSource;
class Arc;

class LayoutItems{
    public:
        static void updateLayout(QGraphicsScene* scene);

    protected:
        LayoutItems(AbstractArcSink*);
        LayoutItems(AbstractArcSource*);
        LayoutItems(Arc*);

        static void unRegisterSinkItem(AbstractArcSink* sink);
        static void unRegisterSourceItem(AbstractArcSource* source);
        static void unRegisterConnection(Arc* arc);

    private:
        static void registerSinkItem(AbstractArcSink* sink);
        static void registerSourceItem(AbstractArcSource* source);
        static void registerConnection(Arc* arc);

        static std::set<AbstractArcSink*> g_sinks;
        static std::set<AbstractArcSource*> g_srcs;
        static std::set<Arc*> g_arcs;
};

class LayoutItem{
    public:
        virtual QGraphicsItem* ultimateParent() = 0;
};

} // namespace liquid

#endif // ndef __LIQUID_LAYOUT_H__

