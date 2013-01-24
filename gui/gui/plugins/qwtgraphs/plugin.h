/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_QWTGRAPHSPLUGIN_H__
#define __CAUV_QWTGRAPHSPLUGIN_H__

#include <gui/core/nodedragging.h>
#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/cauvbasicplugin.h>

#include <QObject>

namespace cauv {
    namespace gui {

        class QwtGraphDropHandler : public DropHandlerInterface<QGraphicsItem * > {

            virtual bool accepts(boost::shared_ptr<Node> const& node);
            virtual QGraphicsItem * handle(boost::shared_ptr<Node> const& node);
        };


        class QwtGraphsPlugin : public QObject, public CauvBasicPlugin
        {
            Q_OBJECT
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

        public:
            virtual const QString name() const;
            virtual void initialise();
            virtual void shutdown();

        protected:
  //          boost::shared_ptr<QwtGraphDropHandler> m_handler;

        };
    } // namespace gui
} // namespace cauv

#endif // __CAUV_QWTGRAPHSPLUGIN_H__
