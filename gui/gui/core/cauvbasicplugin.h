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

#ifndef CAUVBASICPLUGIN_H
#define CAUVBASICPLUGIN_H

#include "cauvplugins.h"

namespace cauv {
    namespace gui {
        class CauvBasicPlugin : public CauvInterfacePlugin
        {
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

        public:
            virtual ~CauvBasicPlugin();

            virtual void initialise(boost::shared_ptr<GuiActions> const& actions);

            virtual void initialise() = 0;

            virtual void shutdown();

        protected:
            boost::shared_ptr<Vehicle> m_auv;
            boost::shared_ptr<GuiActions> m_actions;
        };
    } //namespace gui
}// namespace cauv
#endif // CAUVBASICPLUGIN_H
