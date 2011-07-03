#ifndef MODEL_H
#define MODEL_H

#include "nodes.h"

namespace cauv {

    namespace gui {

        class AUV : public GroupingNode
        {
        public:
            AUV() : GroupingNode("AUV"),
                motors(boost::make_shared<GroupingNode>("motors")),
                autopilots(boost::make_shared<GroupingNode>("autopilots"))
            {
            }

            void populate() {
                this->addChild(motors);
                motors->addChild(boost::make_shared<NumericNode>("prop"));
                motors->addChild(boost::make_shared<NumericNode>("h bow"));
                motors->addChild(boost::make_shared<NumericNode>("v bow"));
                motors->addChild(boost::make_shared<NumericNode>("h stern"));
                motors->addChild(boost::make_shared<NumericNode>("v stern"));

                this->addChild(autopilots);
            }


            // for convienience these are listed as members as well as
            // being in the graph
            boost::shared_ptr<GroupingNode> motors;
            boost::shared_ptr<GroupingNode> autopilots;
        };

    } // namespace gui

} // namespace cauv

#endif // MODEL_H
