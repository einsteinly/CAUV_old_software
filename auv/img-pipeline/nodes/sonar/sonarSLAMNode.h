/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

