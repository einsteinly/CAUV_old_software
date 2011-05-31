#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <boost/shared_ptr.hpp>
#include <common/cauv_node.h>
#include <osg/ref_ptr>
#include <osgViewer/CompositeViewer>

#include "worldmodel.h"

namespace boost {
    namespace program_options {
        class options_description;
        class positional_options_description;
        class variables_map;
    };
}


namespace osg {
    class Group;
}

namespace cauv {

    class AUV;
    class AUVController;

    namespace sim {

        class SimulatedAUV;

        class Simulator : public CauvNode
        {

        public:
            Simulator();

            osg::ref_ptr<cauv::sim::WorldModel> getWorldModel();

            osg::Camera * createHUD();

        protected:
            virtual void onRun();

            virtual void launchViewer();

            int useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc);
            void addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos);

            boost::shared_ptr<AUV> m_auv;
            boost::shared_ptr<AUVController> m_auv_controller;
            boost::shared_ptr<SimulatedAUV> m_simulated_auv;
            osg::ref_ptr<WorldModel> m_world_model;
            osg::ref_ptr<osg::Group> m_root;
            osg::ref_ptr<osgViewer::CompositeViewer> m_viewer;
            osg::Image * shot;

        };

    } // namespace sim

} //namespace cauv

#endif // SIMULATOR_H
