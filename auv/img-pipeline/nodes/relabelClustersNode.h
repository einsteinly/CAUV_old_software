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
            registerInputID("cluster ids");
            registerInputID("image");

            // two outputs:
            registerOutputID("cluster ids");

            // parameters:
            //   K: the number of clusters
            //   colorise: colour each pixel with its clusters centre (otherwise, colour with cluster id)
            registerParamID<int>("C0", 0);
            registerParamID<int>("C1", 0);
            registerParamID<int>("C2", 0);
        }
        
    protected:

        void doWork(in_image_map_t& inputs, out_map_t& r){

            r["cluster ids"] = inputs["cluster ids"];

            cv::Mat clusterIds = inputs["cluster ids"]->mat();
            cv::Mat img = inputs["image"]->mat();
            int target_color[3] = { param<int>("C0"), param<int>("C1"), param<int>("C2") };

            if (clusterIds.type() != CV_8UC1)
                throw parameter_error("Cluster ids must be a 1 channel byte image");

            int y = 0, x = 0, ch = 0;
            int rows = img.rows, cols = img.cols;
            const int img_elem_size = img.elemSize();
            const int img_row_size = img.step[0];
            const int img_channels = img.channels();
            unsigned char *img_rp, *img_cp, *img_bp;

            const int c_row_size = clusterIds.step[0];
            unsigned char *c_rp, *c_bp;
            
            unsigned char best_cluster = 0;
            int best_cluster_sqdist = std::numeric_limits<int>::max();

            // Simultaneously find clusters and find cluster with nearest colour
            for(y = 0, img_rp = img.data, c_rp = clusterIds.data; y < rows; y++, img_rp += img_row_size, c_rp += c_row_size)
                for(x = 0, img_cp = img_rp, c_bp = c_rp; x < cols; x++, img_cp += img_elem_size, ++c_bp)
                {
                    int cluster = *c_bp;

                    int sq_dist = 0;
                    for(ch = 0, img_bp = img_cp; ch < img_channels; ch++, img_bp++)
                    {
                        sq_dist += (target_color[ch] - *img_bp) * (target_color[ch] - *img_bp);
                    }
                    if (sq_dist < best_cluster_sqdist)
                    {
                        best_cluster = cluster;
                        best_cluster_sqdist = sq_dist;
                    }
                }
            
            if (best_cluster == 0)
                return;

            // Relabel clusters to shift best cluster to front (label 0)
            for(y = 0, c_rp = clusterIds.data; y < rows; y++, c_rp += c_row_size)
                for(x = 0, c_bp = c_rp; x < cols; x++, ++c_bp)
                {
                    if (*c_bp == best_cluster)
                        *c_bp = 0;
                    else if (*c_bp < best_cluster)
                        --*c_bp;
                }
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __RELABEL_CLUSTERS_NODE_H__
