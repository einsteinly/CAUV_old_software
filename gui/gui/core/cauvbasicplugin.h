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
            boost::shared_ptr<AUV> m_auv;
            boost::shared_ptr<GuiActions> m_actions;
        };
    } //namespace gui
}// namespace cauv
#endif // CAUVBASICPLUGIN_H
