#ifndef SIMNODE_H
#define SIMNODE_H

#include <vector>
#include <boost/shared_ptr.hpp>

namespace cauv {
    namespace sim {

        class SimNode
        {
        public:
            SimNode();

            virtual void addSimulationChild(boost::shared_ptr<SimNode> node);

            virtual void tick(double simTime);

            virtual void propagateTicks(double simTime);

        protected:
            std::vector<boost::shared_ptr<SimNode> > m_children;
        };

    } // namespace sim
} // namespace cauv

#endif // SIMNODE_H
