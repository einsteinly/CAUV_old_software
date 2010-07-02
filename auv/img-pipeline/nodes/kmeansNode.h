#ifndef __KMEANS_NODE_H__
#define __KMEANS_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_smallint.hpp>
#include <boost/random/variate_generator.hpp>

#include <opencv/cv.h>

#include <common/cauv_utils.h>
#include "../node.h"


class KMeansNode: public Node{
    public:
        KMeansNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t) :
                Node(sched, pl, t),
                m_channels(-1),
                bytedist(0, 255),
                randbyte(gen, bytedist)
        {
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // two outputs:
            registerOutputID<image_ptr_t>("cluster ids");
            registerOutputID<image_ptr_t>("image (not copied)");

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
            
            std::vector<unsigned char> valsum;
        };

        std::vector< cluster > m_clusters;
        int m_channels;
        
        boost::mt19937 gen;
        boost::uniform_smallint<> bytedist;
        boost::variate_generator<boost::mt19937&, boost::uniform_smallint<> > randbyte;

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image"];
            
            int K = param<int>("K");
            bool colorise = !!param<int>("colorise");

            if(K < 1)
            {
                error() << "must be at least one cluster";
                return r;
            }
            else if (K > 255) {
                error() << "too many clusters";
            }

            if (m_channels != img->cvMat().channels()) {
                m_clusters.clear();
                m_channels = img->cvMat().channels();
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

            debug() << "KMeansNode: " << K;
            
            int y, x, ch;
            int rows = img->cvMat().rows, cols = img->cvMat().cols;
            const int elem_size = img->cvMat().elemSize();
            const int row_size = cols * elem_size;
            unsigned char *img_rp, *img_cp, *img_bp;

            cv::Mat clusteridsMat(cols, rows, CV_8UC1);
            boost::shared_ptr<Image> clusterids = boost::make_shared<Image>(clusteridsMat);

            // Clear val sums and sizes (val sum for single pass mean calculation)
            for (size_t i = 0; i < m_clusters.size(); i++) {
                cluster& cl = m_clusters[i];
                for(ch = 0; ch < m_channels; ch++)
                {
                    cl.valsum[ch] = 0;
                    cl.size = 0;
                }
            }

            // Assign each pixel to the nearest cluster
            for(y = 0, img_rp = img->cvMat().data; y < rows; y++, img_rp += row_size)
                for(x = 0, img_cp = img_rp; x < cols; x++, img_cp += elem_size)
                {
                    size_t best_cl_i = UINT_MAX;      
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
                    clusteridsMat.at<unsigned char>(x,y) = clamp_cast<unsigned char>((unsigned char)0, best_cl_i, (unsigned char)255);
                    cluster& best_cl = m_clusters[best_cl_i];
                    for(ch = 0, img_bp = img_cp; ch < m_channels; ch++, img_bp++)
                    {
                        best_cl.valsum[ch] += *img_bp;
                        best_cl.size++;
                    }
                }

            // Change cluster centres (means) based on mean of pixel values
            for (size_t i = 0; i < m_clusters.size(); i++) {
                cluster& cl = m_clusters[i];
                for(ch = 0; ch < m_channels; ch++)
                {
                    cl.centre[ch] = cl.valsum[ch]/cl.size;
                }
            }

            if (colorise) {
                // Colorise if necessary

                for(y = 0, img_rp = img->cvMat().data; y < rows; y++, img_rp += row_size)
                    for(x = 0, img_cp = img_rp; x < cols; x++, img_cp += elem_size)
                    {
                        cluster& cl = m_clusters[clusteridsMat.at<unsigned char>(x,y)];

                        for (size_t i = 0; i < m_clusters.size(); i++) {
                            *img_bp = cl.centre[ch];
                        }
                    }
            }
            
            r["cluster ids"] = clusterids;
            r["image (not copied)"] = img;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

#endif // ndef __GAUSSIAN_BLUR_NODE_H__
