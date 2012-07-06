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

#ifndef __KMEANS_NODE_H__
#define __KMEANS_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_smallint.hpp>
#include <boost/random/variate_generator.hpp>

#include <opencv2/core/core.hpp>

#include <utility/rounding.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class KMeansNode: public Node{
    public:
        KMeansNode(ConstructArgs const& args) :
                Node(args),
                m_channels(-1),
                bytedist(0, 255),
                randbyte(gen, bytedist)
        {
        }

        void init()
        {
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // two outputs:
            registerOutputID("clusters", std::vector<Colour>());
            registerOutputID("labels");
            registerOutputID("image (not copied)");

            // parameters:
            //   K: the number of clusters
            //   colorise: colour each pixel with its clusters centre (otherwise, colour with cluster id)
            registerParamID<int>("K", 5);
            registerParamID<int>("colorise", 1);
        }
        
    protected:
        struct cluster
        {
            std::vector<unsigned char> centre;
            unsigned int size;
            
            std::vector<int> valsum;
        };

        std::vector< cluster > m_clusters;
        int m_channels;
        
        boost::mt19937 gen;
        boost::uniform_smallint<> bytedist;
        boost::variate_generator<boost::mt19937&, boost::uniform_smallint<> > randbyte;

        // Don't be surprised, this one does one iteration, but keeps the result between frames
        void doWork(in_image_map_t& inputs, out_map_t& r){

            cv::Mat img = inputs["image"]->mat();
            
            int K = param<int>("K");
            bool colorise = !!param<int>("colorise");

            if(K < 1) {
                error() << "must be at least one cluster";
            }
            else if (K > 255) {
                error() << "too many clusters";
            }

            if (m_channels != img.channels()) {
                m_clusters.clear();
                m_channels = img.channels();
            }

            // Make sure our number of clusters == K, adding or removing as appropriate
            int sizediff = m_clusters.size() - K;
            if (sizediff < 0) {
                for (int i = 0; i < -sizediff; i++) {
                    cluster cl;
                    // Random cluster centre
                    for (int c = 0; c < m_channels; c++) {
                        cl.centre.push_back(randbyte());
                        cl.valsum.push_back(0);
                    }
                    cl.size = 0;
                    m_clusters.push_back(cl);
                }
            }
            else if (sizediff > 0) {
                for (int i = 0; i < sizediff; i++) {
                    unsigned int min_cl_size = UINT_MAX;
                    size_t min_cl_i = UINT_MAX;      
                    // Remove the smallest clusters
                    for (size_t j = 0; j < m_clusters.size(); j++) {
                        cluster& cl = m_clusters[i];
                        if (cl.size < min_cl_size) {
                            min_cl_size = cl.size;
                            min_cl_i = j;
                        }
                    }
                    assert(min_cl_i != UINT_MAX);
                    m_clusters.erase(m_clusters.begin()+min_cl_i);
                }
            }

            int y = 0, x = 0, ch = 0;
            int rows = img.rows, cols = img.cols;
            const int elem_size = img.elemSize();
            const int row_size = img.step[0];
            unsigned char *img_rp, *img_cp, *img_bp;

            cv::Mat_<unsigned char> clusteridsMat(rows, cols);

            // Clear val sums and sizes (val sum for single pass mean calculation)
            for (size_t i = 0; i < m_clusters.size(); i++) {
                cluster& cl = m_clusters[i];
                for(ch = 0; ch < m_channels; ch++)
                {
                    cl.valsum[ch] = 0;
                }
                cl.size = 0;
            }

            // Assign each pixel to the nearest cluster
            for(y = 0, img_rp = img.data; y < rows; y++, img_rp += row_size)
                for(x = 0, img_cp = img_rp; x < cols; x++, img_cp += elem_size)
                {
                    size_t best_cl_i = K;
                    unsigned int best_cl_sqdiff = UINT_MAX;      
                    for (size_t i = 0; i < m_clusters.size(); i++) {
                        cluster& cl = m_clusters[i];
                        unsigned int sqdiff = 0;
                        for(ch = 0, img_bp = img_cp; ch < m_channels; ch++, img_bp++)
                        {
                            sqdiff += (*img_bp - cl.centre[ch]) * (*img_bp - cl.centre[ch]);
                        }

                        if (sqdiff < best_cl_sqdiff) {
                            best_cl_i = i;
                            best_cl_sqdiff = sqdiff;
                        }
                    }
                    assert(best_cl_i < (unsigned int)K);
                    clusteridsMat.at<unsigned char>(y,x) = clamp_cast<unsigned char>((unsigned char)0, best_cl_i, (unsigned char)255);
                    cluster& best_cl = m_clusters[best_cl_i];
                    best_cl.size++;
                    for(ch = 0, img_bp = img_cp; ch < m_channels; ch++, img_bp++)
                    {
                        best_cl.valsum[ch] += *img_bp;
                    }
                }

            // Find empty clusters, and assign to them the most distant point
            for (size_t i = 0; i < m_clusters.size(); i++)
            {
                cluster& cl = m_clusters[i];
                if (cl.size == 0)
                {
                    int farthest_point_x = -1;
                    int farthest_point_y = -1;
                    unsigned int farthest_point_sqdist = 0;

                    for(y = 0, img_rp = img.data; y < rows; y++, img_rp += row_size)
                        for(x = 0, img_cp = img_rp; x < cols; x++, img_cp += elem_size)
                        {
                            cluster& other_cl = m_clusters[clusteridsMat.at<unsigned char>(y,x)];
                            if(other_cl.size < 2)
                            {
                                continue;
                            }
                            
                            unsigned int sqdist = 0;
                            for(ch = 0, img_bp = img_cp; ch < m_channels; ch++, img_bp++)
                            {
                                sqdist += (*img_bp - other_cl.centre[ch]) * (*img_bp - other_cl.centre[ch]);
                            }
                            
                            if (sqdist >= farthest_point_sqdist) {
                                farthest_point_sqdist = sqdist;
                                farthest_point_x = x;
                                farthest_point_y = y;
                            }
                        }
                   
                    assert(farthest_point_x != -1 && farthest_point_y != -1);
                    
                    int farthest_point_clid = clusteridsMat.at<unsigned char>(farthest_point_y,farthest_point_x);
                    cluster& farthest_point_cl = m_clusters[farthest_point_clid];
                    clusteridsMat.at<unsigned char>(farthest_point_y,farthest_point_x) = i;
                    
                    farthest_point_cl.size--;
                    cl.size++;
                    for(ch = 0, img_bp = img.data + farthest_point_y * row_size + farthest_point_x * elem_size; ch < m_channels; ch++, img_bp++)
                    {
                        farthest_point_cl.valsum[ch] -= *img_bp;
                        cl.valsum[ch] += *img_bp;
                    }
                }
            }


            // Change cluster centres (means) based on mean of pixel values
            for (size_t i = 0; i < m_clusters.size(); i++)
            {
                cluster& cl = m_clusters[i];
                assert(cl.size != 0);
                
                for(ch = 0; ch < m_channels; ch++)
                {
                    cl.centre[ch] = cl.valsum[ch]/cl.size;
                    debug(5) << i << ".centre[" << ch << "] =" << (int) cl.centre[ch];
                }
            }

            if(hasChildOnOutput("clusters")) {
                std::vector<Colour> clusters;
                for (size_t i = 0; i < m_clusters.size(); i++)
                {
                    cluster& cl = m_clusters[i];
                    if (m_channels == 3)
                        clusters.push_back(Colour::fromBGR(cl.centre[0]/255.0f, cl.centre[1]/255.0f, cl.centre[2]/255.0f));
                    else if (m_channels == 1)
                        clusters.push_back(Colour::fromGrey(cl.centre[0]/255.0f));
                    else
                        error() << "No colour for" << m_channels << " channel image";
                }
                r["clusters"] = clusters;
            }


            if (colorise) {
                // Colorise if necessary

                for(y = 0, img_rp = img.data; y < rows; y++, img_rp += row_size)
                    for(x = 0, img_cp = img_rp; x < cols; x++, img_cp += elem_size)
                    {
                        cluster& cl = m_clusters[clusteridsMat.at<unsigned char>(y,x)];

                        for(ch = 0, img_bp = img_cp; ch < m_channels; ch++, img_bp++)
                        {
                            *img_bp = cl.centre[ch];
                        }
                    }
            }
            
            r["labels"] = boost::make_shared<Image>(clusteridsMat);
            r["image (not copied)"] = boost::make_shared<Image>(img);
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __KMEANS_NODE_H__
