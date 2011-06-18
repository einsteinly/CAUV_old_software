#ifndef SIMNODE_H
#define SIMNODE_H

#include <vector>
#include <boost/shared_ptr.hpp>

#include <sim/simulator.h>

namespace cauv {
    namespace sim {

        class SimNode
        {
        public:
            SimNode(Simulator * s);

            virtual void addSimulationChild(boost::shared_ptr<SimNode> node);

            virtual void tick(double simTime);

            virtual void propagateTicks(double simTime);

        protected:
            std::vector<boost::shared_ptr<SimNode> > m_children;
            Simulator * m_simulator;
        };

    } // namespace sim
} // namespace cauv

#endif // SIMNODE_H
