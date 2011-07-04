#ifndef MODEL_H
#define MODEL_H

#include "nodes.h"

namespace cauv {

    namespace gui {

        class AUV : public GroupingNode
        {
        public:

            AUV() : GroupingNode("AUV")
                //motors(),
                //autopilots(boost::make_shared<GroupingNode>("autopilots"))
            {
            }

            void populate() {
                /*
                boost::shared_ptr<GroupingNode> motorsNode = boost::make_shared<GroupingNode>("motors");
                addChild(motorsNode);
                motorsNode->addChild(motors[MotorID::Prop] = boost::make_shared<NumericNode>("prop"));
                motorsNode->addChild(motors[MotorID::HBow] = boost::make_shared<NumericNode>("h bow"));
                motorsNode->addChild(motors[MotorID::VBow] = boost::make_shared<NumericNode>("v bow"));
                motorsNode->addChild(motors[MotorID::HStern] = boost::make_shared<NumericNode>("h stern"));
                motorsNode->addChild(motors[MotorID::VStern] = boost::make_shared<NumericNode>("v stern"));

                this->addChild(autopilots);
                addAutopilot("bearing");
                addAutopilot("depth");
                addAutopilot("pitch");*/
            }
/*
            boost::shared_ptr<GroupingNode> addAutopilot(std::string name){
                boost::shared_ptr<GroupingNode> autopilot = boost::make_shared<GroupingNode>(name);

                autopilot->addChild(boost::make_shared<NumericNode>("target"));
                autopilot->addChild(boost::make_shared<NumericNode>("enabled"));

                autopilot->addChild(boost::make_shared<NumericNode>("Kp"));
                autopilot->addChild(boost::make_shared<NumericNode>("Ki"));
                autopilot->addChild(boost::make_shared<NumericNode>("Kd"));
                autopilot->addChild(boost::make_shared<NumericNode>("scale"));
                autopilot->addChild(boost::make_shared<NumericNode>("Ap"));
                autopilot->addChild(boost::make_shared<NumericNode>("Ai"));
                autopilot->addChild(boost::make_shared<NumericNode>("Ad"));
                autopilot->addChild(boost::make_shared<NumericNode>("thr"));
                autopilot->addChild(boost::make_shared<NumericNode>("maxError"));

                return autopilot;
            }


            // just for convienience
            typedef std::map<MotorID::e, boost::shared_ptr<NumericNode> > motor_map;
            motor_map motors;

            boost::shared_ptr<GroupingNode> autopilots;*/
        };

    } // namespace gui

} // namespace cauv

#endif // MODEL_H
