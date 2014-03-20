/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef GUI_VEHICLENODE_H
#define GUI_VEHICLENODE_H

#include <QObject>

#include "model/node.h"

namespace cauv {
    namespace gui {

        class Vehicle : public Node
        {
            Q_OBJECT

        public:
            friend class VehicleRegistry;

        protected:
            Vehicle(const std::string& name) : Node(name, nodeType<Vehicle>()) {
            }

            virtual void initialise() = 0;
#if 0
        Q_SIGNALS:
            //void messageGenerated(boost::shared_ptr<const Message>);
            void observerAttached(boost::shared_ptr<BaseMessageHandler>);
            void observerDetached(boost::shared_ptr<BaseMessageHandler>);

        public:

            typedef std::set<boost::shared_ptr<BaseMessageGenerator> > generator_set_t;
            typedef std::set<boost::shared_ptr<BaseMessageHandler> > observer_set_t;

            void attachGenerator(boost::shared_ptr<BaseMessageGenerator> generator)
            {
                connect(generator->node().get(), SIGNAL(detachedFrom(boost::shared_ptr<Node>)),
                        this, SLOT(nodeRemoved()));

                //generator->connect(generator.get(), SIGNAL(messageGenerated(boost::shared_ptr<const Message>)),
                //                   this, SIGNAL(messageGenerated(boost::shared_ptr<const Message>)));
                if(dynamic_cast<BaseMessageHandler*>(generator.get())) {
                    attachObserver(generator->node(), boost::dynamic_pointer_cast<BaseMessageHandler>(generator));
                }

                m_generators[generator->node()].insert(generator);
            }

            void detachGenerators(boost::shared_ptr<Node> node){
                CAUV_LOG_INFO("detach generators");
                foreach(boost::shared_ptr<BaseMessageGenerator> const& generator, m_generators[node]){
                    CAUV_LOG_INFO("generator found");
                    m_generators[node].erase(generator);
                }
                m_generators.erase(node);
            }

            void attachObserver(boost::shared_ptr<Node> node,
                                boost::shared_ptr<BaseMessageHandler> observer) {
                m_observers[node].insert(observer);
                Q_EMIT observerAttached(observer);
            }

            void detachObservers(boost::shared_ptr<Node> node) {
                CAUV_LOG_INFO("detach observers");
                foreach(boost::shared_ptr<BaseMessageHandler> const& observer, m_observers[node]){
                    CAUV_LOG_INFO("observer found");
                    m_observers[node].erase(observer);
                    Q_EMIT observerDetached(observer);
                }
                m_observers.erase(node);
            }
#endif
        protected Q_SLOTS:
            void nodeRemoved() {
                CAUV_LOG_INFO("node removed");
#if 0
                if(Node* node = dynamic_cast<Node*>(sender())) {
                    detachGenerators(node->shared_from_this());
                    detachObservers(node->shared_from_this());
                }
#endif
            }
#if 0
        protected:
            std::map<boost::shared_ptr<Node>,  generator_set_t > m_generators;
            std::map<boost::shared_ptr<Node>,  observer_set_t > m_observers;
#endif
        };

    } //namespace gui
} // namespace cauv

#endif // GUI_VEHICLENODE_H
