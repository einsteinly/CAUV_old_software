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

#ifndef __RELABEL_CLUSTERS_NODE_H__
#define __RELABEL_CLUSTERS_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../node.h"
#include "../nodeFactory.h"
#include "../pipelineTypes.h"


namespace cauv{
namespace imgproc{

class RelabelClustersNode: public Node{
    public:
        RelabelClustersNode(ConstructArgs const& args) :
                Node(args)
        {
        }

        void init()
        {
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("labels");

            // two outputs:
            registerOutputID("clusters", std::vector<Colour>());
            registerOutputID("labels (not copied)");

            // parameters:
            registerParamID("clusters", std::vector<Colour>());
            registerParamID<Colour>("colour", Colour::fromBGR(0,0,0));
        }
        
    protected:

        struct colourICmp {
            const std::vector<float>& dists;
            colourICmp(const std::vector<float>& dists) : dists(dists) {}
            bool operator()(const size_t& i1, const size_t& i2) {
                return dists[i1] < dists[i2];
            }
        };


        void doWork(in_image_map_t& inputs, out_map_t& r){

            std::vector<Colour> clusters = param<std::vector<Colour> >("clusters");
            Colour colour = param<Colour>("colour");
            
            std::vector<float> clusterDists;
            std::vector<size_t> clusterIs;

            for(size_t i = 0; i < clusters.size(); ++i) {
                const Colour& cluster_colour = clusters[i];
                float sqdiff = cauv::sqdiff(colour, cluster_colour);
                clusterDists.push_back(sqdiff);
                clusterIs.push_back(i);
            }
            std::sort(clusterIs.begin(), clusterIs.end(), colourICmp(clusterDists));
            std::vector<Colour> new_clusters;
            for(size_t i = 0; i < clusterIs.size(); ++i) {
                new_clusters.push_back(clusters[clusterIs[i]]);
            }
                
            r["clusters"] = new_clusters;

            if(hasChildOnOutput("labels (not copied)")) {
                cv::Mat labels = inputs["labels"]->mat();
                if (labels.type() != CV_8UC1)
                    throw parameter_error("Labels must be a 1 channel byte image");

                cv::MatIterator_<unsigned char> it, itend;
                for (it = labels.begin<unsigned char>(), itend = labels.end<unsigned char>(); it != itend; ++it) {
                    *it = clusterIs[*it];
                }
                r["labels (not copied)"] = inputs["labels"];
            }
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __RELABEL_CLUSTERS_NODE_H__
