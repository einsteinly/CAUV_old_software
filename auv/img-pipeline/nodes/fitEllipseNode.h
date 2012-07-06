/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef __FITELLIPSE_H__
#define __FITELLIPSE_H__

#include "../node.h"
#include "../nodeFactory.h"
#include "outputNode.h"

namespace cauv{
namespace imgproc{

class FitEllipseNode : public Node {
    public:
        FitEllipseNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            //Fast node
            m_speed = fast;
            
            //One input
            registerInputID("edge_image");
            registerInputID("imageDx");
            registerInputID("imageDy");
            
            //One output
            registerOutputID("ellipse", std::vector<Ellipse>());
            
            //Parameters
            registerParamID<int>("minRadius", 10, "minimum ellipse radius");
            registerParamID<int>("maxRadius", 60, "maximum ellipse radius");
            registerParamID<int>("maxIters", 1000, "maximum RANSAC iterations");
            
        }

    protected:
        virtual void doWork(in_image_map_t& inputs, out_map_t& r);

    //Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif //ndef __FITELLIPSE_H__
