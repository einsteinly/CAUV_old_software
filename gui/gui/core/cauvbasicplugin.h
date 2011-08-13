#ifndef CAUVBASICPLUGIN_H
#define CAUVBASICPLUGIN_H

#include "cauvplugins.h"

namespace cauv {
    namespace gui {
        class CauvBasicPlugin : public CauvInterfacePlugin
        {
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

        public:
            virtual ~CauvBasicPlugin(){}

            virtual void initialise(boost::shared_ptr<GuiActions> actions);

            virtual void initialise() = 0;

        protected:
            boost::shared_ptr<AUV> m_auv;
            boost::shared_ptr<CauvNode> m_node;
            boost::shared_ptr<GuiActions> m_actions;
        };
    } //namespace gui
}// namespace cauv
#endif // CAUVBASICPLUGIN_H
