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

#ifndef AIPLUGIN_H
#define AIPLUGIN_H

#include <gui/core/cauvbasicplugin.h>
#include <gui/core/model/node.h>

#include <QObject>

namespace cauv {
    namespace gui {

        class NodeChildrenExclusionFilter;

        class AiPlugin : public QObject, public CauvBasicPlugin
        {
            Q_OBJECT
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

        public:
            AiPlugin();
            virtual const QString name() const;
            virtual void initialise();

        public Q_SLOTS:
            void setupTask(boost::shared_ptr<Node> node);
            void setupCondition(boost::shared_ptr<Node> node);
            void setupVehicle(boost::shared_ptr<Node> node);
            void reloadAi();

        protected:
            boost::shared_ptr<NodeChildrenExclusionFilter> m_filter;

        };


        class ReloadAiFilter : public QObject {
            Q_OBJECT
        public:
            virtual bool eventFilter(QObject *, QEvent *event){
                if (event->type() == QEvent::KeyPress) {
                    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
                    if (keyEvent->key() == Qt::Key_R) {
                        Q_EMIT reloadAiRequest();
                        return true;
                    } else
                        return false;
                }
                return false;
            }

        Q_SIGNALS:
            void reloadAiRequest();
        };


    } // namespace gui
} // namespace cauv

#endif // AIPLUGIN_H
