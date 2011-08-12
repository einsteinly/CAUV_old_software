#ifndef DROPHANDLER_H
#define DROPHANDLER_H

#include <boost/shared_ptr.hpp>
#include "../model/nodes_fwd.h"

#include <QGraphicsItem>

namespace cauv {
    namespace gui {

        struct drop_not_handled : public std::exception {};

        class DropHandler
        {
        public:
            DropHandler();

            template <class T> bool handles(boost::shared_ptr<T> node){
                try {
                    QGraphicsItem * item = handle(node);
                    delete item;
                    return true;
                } catch (drop_not_handled){}
                return false;
            }

            virtual QGraphicsItem * handle(boost::shared_ptr<NumericNode> );
            virtual QGraphicsItem * handle(boost::shared_ptr<ImageNode> );
            virtual QGraphicsItem * handle(boost::shared_ptr<FloatYPRNode> );
            virtual QGraphicsItem * handle(boost::shared_ptr<FloatXYZNode> );
            virtual QGraphicsItem * handle(boost::shared_ptr<GroupingNode> );
        };

    } // namespace gui
} // namespace cauv

#endif // DROPHANDLER_H
