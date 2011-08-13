#ifndef DROPHANDLER_H
#define DROPHANDLER_H

#include <boost/shared_ptr.hpp>
#include "../model/nodes_fwd.h"

#include <QGraphicsItem>

namespace cauv {
    namespace gui {

        struct drop_not_handled : public std::exception {};

        template <class T>
        class DropHandlerInterface {
        public:
            // return true if the object will handle the node, but
            // don't actually handle it yet
            virtual bool willHandle(boost::shared_ptr<NodeBase> node) = 0;

            // return some object that the caller is interested in
            // if passed an obejct you're not interested in throw a
            // drop_not_handled exception
            virtual T handle(boost::shared_ptr<NodeBase> node) = 0;
        };

    } // namespace gui
} // namespace cauv

#endif // DROPHANDLER_H
