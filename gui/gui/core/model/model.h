#ifndef GUI_MODEL_H
#define GUI_MODEL_H

#include "node.h"

#include <generated/types/message.h>

#include "../model/nodes/groupingnode.h"

namespace cauv {
    namespace gui {

        class MessageGenerator;

        class AUV : public GroupingNode
        {
            Q_OBJECT
        public:
            AUV(std::string name) : GroupingNode(name) {
            }

            virtual void initialise() = 0;

        Q_SIGNALS:
            void messageGenerated(boost::shared_ptr<const Message>);

        protected:
            void addGenerator(boost::shared_ptr<MessageGenerator> generator);

            std::vector<boost::shared_ptr<MessageGenerator> > m_generators;
        };


        class RedHerring : public AUV
        {
            Q_OBJECT
        public:
            RedHerring();
            virtual void initialise();

        protected Q_SLOTS:
            void setupMotor(boost::shared_ptr<NodeBase>);
            void setupAutopilot(boost::shared_ptr<NodeBase> node);
        };

    } // namespace gui
} // namespace cauv

#endif // GUI_MODEL_H
