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

#include <gui/core/cauvbasicplugin.h>
#include <gui/core/cauvfluidityplugin.h>
#include <gui/core/model/node.h>

#include <QObject>

namespace cauv{
class CauvNode;

namespace gui{

class NodeChildrenExclusionFilter;

/* Only one FluidityPlugin per process. An exception will be thrown from the
 * initialise function if more than one FluidityPlugin is created and
 * initialised in the same process.
 * (It would be possible to relax this constraint a little)
 */
class FluidityPlugin: public QObject,
                      public CauvBasicPlugin,
                      public FluidityPluginInterface
{
    Q_OBJECT
    Q_INTERFACES(cauv::gui::CauvInterfacePlugin)
    Q_INTERFACES(cauv::gui::FluidityPluginInterface)

    public:
        FluidityPlugin();
        virtual const QString name() const;
        virtual void initialise();
        
        // FluidityPluginInterface
        LiquidFluidityNode* newLiquidNodeFor(boost::shared_ptr<FluidityNode> node);

    //public Q_SLOTS:

    protected:

    private:
        friend class LiquidFluidityNode;
        
        // this is set by the first instance of a FluidityPlugin to be initialised.
        static boost::weak_ptr<CauvNode>& theCauvNode();
        
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_FUIDITY_PLUGIN_H__
