/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

            virtual void initialise(boost::shared_ptr<GuiActions> const& actions, ConnectedNodeMap*);

            virtual void shutdown();

        protected:

            virtual void initialise() = 0;

            boost::shared_ptr<GuiActions> m_actions;
        };
    } //namespace gui
}// namespace cauv
#endif // CAUVBASICPLUGIN_H
