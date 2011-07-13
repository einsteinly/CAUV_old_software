#ifndef GUI_MODEL_H
#define GUI_MODEL_H

#include "nodes.h"

namespace cauv {
    namespace gui {

        class AUV : public GroupingNode
        {
        public:
            AUV() : GroupingNode("redherring") {
            }
        };

    } // namespace gui
} // namespace cauv

#endif // GUI_MODEL_H
