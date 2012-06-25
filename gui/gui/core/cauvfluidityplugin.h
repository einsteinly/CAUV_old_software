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

#ifndef __CAUV_FLUIDITY_PLUGIN_INTERFACE_H__
#define __CAUV_FLUIDITY_PLUGIN_INTERFACE_H__

#include <boost/shared_ptr.hpp>

namespace cauv{
namespace gui{

class FluidityNode;
class LiquidFluidityNode;

class FluidityPluginInterface{
    public:
        virtual LiquidFluidityNode* newLiquidNodeFor(boost::shared_ptr<FluidityNode> node) = 0;
        virtual ~FluidityPluginInterface(){ }
};

} // namespace gui
} // namespace cauv

Q_DECLARE_INTERFACE(cauv::gui::FluidityPluginInterface, "co.uk.cauv.FluidityPluginInterface/1.0")

#endif // __CAUV_FLUIDITY_PLUGIN_INTERFACE_H__
