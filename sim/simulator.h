/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <boost/shared_ptr.hpp>
#include <common/cauv_node.h>
#include <osg/ref_ptr>
#include <osgViewer/Viewer>

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
            osg::ref_ptr<osgViewer::Viewer> m_viewer;
            osg::Image * shot;

        };

    } // namespace sim

} //namespace cauv

#endif // SIMULATOR_H
