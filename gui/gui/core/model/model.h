#ifndef MODEL_H
#define MODEL_H

#include "nodes.h"

namespace cauv {

    namespace gui {

        class AUV : public GroupingNode
        {
        public:
            AUV() : GroupingNode("AUV") {
            }
        };

    } // namespace gui
} // namespace cauv

#endif // MODEL_H
