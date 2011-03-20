#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <boost/shared_ptr.hpp>
#include <common/cauv_node.h>
#include <osg/ref_ptr>

#include "worldmodel.h"

namespace boost {
    namespace program_options {
        class options_description;
        class positional_options_description;
        class variables_map;
    };
}


namespace osg {
    class Node;
}


namespace cauv {

    class AUV;
    class AUVController;

    class Simulator : public CauvNode
    {

    public:
        Simulator();

        osg::ref_ptr<cauv::WorldModel> getWorldModel();

    protected:
        virtual void onRun();

        virtual void launchViewer(osg::ref_ptr<osg::Node> root);

        int useOptionsMap(boost::program_options::variables_map& vm, boost::program_options::options_description& desc);
        void addOptions(boost::program_options::options_description& desc, boost::program_options::positional_options_description& pos);

        boost::shared_ptr<AUV> m_auv;
        boost::shared_ptr<AUVController> m_auv_controller;
        osg::ref_ptr<WorldModel> m_world_model;
    };

} //namespace cauv

#endif // SIMULATOR_H
