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

#ifndef __SONAR_SLAM_NODE_H__
#define __SONAR_SLAM_NODE_H__

#include "../outputNode.h"
#include "../../nodeFactory.h"

namespace cauv{
namespace imgproc{

class SonarSLAMImpl;

class SonarSLAMNode: public OutputNode{
    public:
        SonarSLAMNode(ConstructArgs const& args) : OutputNode(args){ }

        void init();

        virtual bool requiresGPS() const { return true; }
        virtual bool requiresTelemetry() const { return true; }

        virtual void onGPSLoc(boost::shared_ptr<GPSLocationMessage const>);
        virtual void onTelemetry(boost::shared_ptr<TelemetryMessage const>);

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r);

    private:
        boost::shared_ptr<SonarSLAMImpl> m_impl;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SONAR_SLAM_NODE_H__

