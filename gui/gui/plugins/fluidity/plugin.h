/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#ifndef __CAUV_FUIDITY_PLUGIN_H__
#define __CAUV_FUIDITY_PLUGIN_H__

#include <QObject>

#include <cauvbasicplugin.h>

#include <model/node.h>
#include <model/nodes/groupingnode.h>
#include <model/nodes/vehiclenode.h>

#include <liquid/node.h>

namespace cauv{
class CauvNode;

namespace gui{

class FluidityNode;

class NodeChildrenExclusionFilter;

/* Only one FluidityPlugin per process. An exception will be thrown from the
 * initialise function if more than one FluidityPlugin is created and
 * initialised in the same process.
 * (It would be possible to relax this constraint a little)
 */
class FluidityPlugin: public QObject,
        public CauvBasicPlugin
{
    Q_OBJECT
    Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

public:
    FluidityPlugin();
    virtual const QString name() const;
    virtual void initialise();

public Q_SLOTS:
    void setupVehicle(boost::shared_ptr<Node> node);
    void setupPipeline(boost::shared_ptr<Node> node);
    void onSubscribed(MessageType::e messageType);

private:
    friend class LiquidFluidityNode;

    // this is set by the first instance of a FluidityPlugin to be initialised.
    static boost::weak_ptr<CauvNode>& theCauvNode();


};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_FUIDITY_PLUGIN_H__
