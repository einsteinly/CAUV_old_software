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

#ifndef __FAST_MEDIAN_NODE_H__
#define __FAST_MEDIAN_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class FastMedianNode: public Node{
    public:
        FastMedianNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");

            // one output
            registerOutputID("image");
            
            // parameter: 
            registerParamID<float>("radius", 5, "filter aperture (odd values only)");
        }
    
        virtual ~FastMedianNode(){
            stop();
        }

    protected:
        struct HistAccumulator{
            HistAccumulator()
                : m_counts(256, 0), m_total(0){
            }
            void add(uint8_t v){
                m_counts[v]++;
                m_total++;
            }
            void rem(uint8_t v){
                m_counts[v]--;
                m_total--;
                #ifndef CAUV_NO_DEBUG
                assert(m_counts[v] >= 0);
                assert(m_total < 4e9);
                #endif
            }
            uint8_t median() const{
                unsigned t = 0;
                for(unsigned i = 0; i != m_counts.size(); i++){
                    t += m_counts[i];
                    if(t >= (m_total+1)/2)
                        return i;
                }
                error() << "median error";
                return 0;
            }
            
            std::vector<int> m_counts;
            unsigned m_total;
        };
    
        /* For circular apertures, pre-calculate the indices of pixels that
         * need to be added/removed as the aperture is moved
         *
         *  . . . . . . . . .
         *  . . - o o + . . .  (not actually the kernel shape for r=2)
         *  . - o o o o + . .
         *  . - o o o o + . .
         *  . - o o o o + . .
         *  . . - o o + . . .
         *  . . . . . . . . .
         *          |---|
         *            radius = 2
         * Only one side (the + markers, above) are returned, since the other
         * (and top/bottom) can easily be worked out by symmetry.
         * 
         */
        static std::vector<int> calcApertureDiff(int r){
            std::vector<int> ret;
            int comp = int((r+0.5)*(r+0.5)+0.5);
            for(int i = -r; i <= r; i++)
                for(int j = 0; j <= r; j++)
                    if(i*i + j*j <= comp && i*i + (j+1)*(j+1) > comp)
                        ret.push_back(j);
            assert(int(ret.size()) == 2*r+1);
            return ret;
        }
        struct applyFastMedian: boost::static_visitor<augmented_mat_t>{
            applyFastMedian(float sigma) : m_sigma(sigma){ }
            augmented_mat_t operator()(cv::Mat a) const{
                int radius = int(m_sigma+0.5);
                 
                if(!a.isContinuous())
                    throw(parameter_error("image must be continuous"));
                if((a.type() & CV_MAT_DEPTH_MASK) != CV_8U)
                    throw(parameter_error("image must be unsigned bytes"));
                if(a.channels() > 3)
                    throw(parameter_error("image must be <= 3-channel"));
                    // TODO: support vector parameters
                
                const int channels = a.channels();
                const int rows = a.rows;
                const int cols = a.cols;
                const int elem_size = a.elemSize();
                const std::vector<int> ad = calcApertureDiff(radius);
                const cv::Mat& imat = a;
                
                std::vector< HistAccumulator > accum(3, HistAccumulator());
                cv::Mat output(imat.size(), imat.type());
                
                /* Move the filter kernel over the image, maintaining per-channel
                 * histograms from which the median can easily be calculated:
                 *
                 *  radius                 ol = cols -1
                 * |======|                   |
                 * o--------------------------> -- row = 0
                 *        <--------------------
                 *        -------------------->
                 *        <--------------------
                 *        -------------------->
                 *        <--------------------
                 *     -- --------------------> -- row = rows-1
                 *        |                   |
                 *    col = 0
                 */
                
                int row, col, ch, krow, kcol;
                // lead-in:
                for(col = -radius; col <= 0; col++)
                    for(ch = 0; ch < channels; ch++)
                        for(krow = 0; krow <= radius; krow++){
                            const int aprow = radius+krow;
                            if(col + ad[aprow] >= 0)
                                accum[ch].add(*(imat.ptr(krow) + (col +ad[aprow])*elem_size +ch));
                        }
                col = 0;
                for(row = 0; row < rows; row++){
                    if(row & 1){
                        // <--------------
                        assert(col == cols-1);
                        for(; col > 0; col--)
                            for(ch = channels-1; ch >= 0; ch--){
                                *(output.ptr(row) + col*elem_size + ch) = accum[ch].median();
                                for(krow = clamp(0, row-radius, rows); krow < clamp(0, row+radius+1, rows); krow++){
                                    const int aprow = radius+krow-row;
                                    if(col + ad[aprow] < cols)
                                        accum[ch].rem(*(imat.ptr(krow) + (col + ad[aprow])*elem_size + ch));
                                    if(col - ad[aprow] - 1 >= 0)
                                        accum[ch].add(*(imat.ptr(krow) + (col - ad[aprow] - 1)*elem_size + ch));
                                }
                            }
                        for(ch = channels-1; ch >= 0; ch--)
                            *(output.ptr(row) + col*elem_size + ch) = accum[ch].median();
                        assert(col == 0);
                        // move down one row
                        for(kcol = 0; kcol <= clamp(0, radius, cols); kcol++){
                            const int apcol = radius+kcol-col;
                            if(row - ad[apcol] >=0)
                                for(ch = 0; ch < channels; ch++)
                                    accum[ch].rem(*(imat.ptr(row - ad[apcol]) + kcol*elem_size + ch));
                            if(row + ad[apcol] + 1 < rows)
                                for(ch = 0; ch < channels; ch++)
                                    accum[ch].add(*(imat.ptr(row + ad[apcol] + 1) + kcol*elem_size + ch));
                        }
                    }else{
                        // --------------->
                        assert(col == 0);
                        for(; col < cols-1; col++)
                            for(ch = 0; ch < channels; ch++){
                                *(output.ptr(row) + col*elem_size + ch) = accum[ch].median();
                                for(krow = clamp(0, row-radius, rows); krow < clamp(0, row+radius+1, rows); krow++){
                                    const int aprow = radius+krow-row;
                                    if(col - ad[aprow] >= 0)
                                        accum[ch].rem(*(imat.ptr(krow) + (col - ad[aprow])*elem_size + ch));
                                    if(col + ad[aprow] + 1 < cols)
                                        accum[ch].add(*(imat.ptr(krow) + (col + ad[aprow] + 1)*elem_size + ch));
                                }
                            }
                        for(ch = 0; ch < channels; ch++)
                            *(output.ptr(row) + col*elem_size + ch) = accum[ch].median();
                        assert(col == cols-1);
                        // move down one row
                        for(kcol = col; kcol >= clamp(0, col-radius, cols); kcol--){
                            const int apcol = radius+kcol-col;
                            if(row - ad[apcol] >=0)
                                for(ch = 0; ch < channels; ch++)
                                    accum[ch].rem(*(imat.ptr(row - ad[apcol]) + kcol*elem_size + ch));
                            if(row + ad[apcol] + 1 < rows)
                                for(ch = 0; ch < channels; ch++)
                                    accum[ch].add(*(imat.ptr(row + ad[apcol] + 1) + kcol*elem_size + ch));
                        }
                    }
                }
                return output;
            }
            augmented_mat_t operator()(NonUniformPolarMat a) const{
                // TODO: might want to filter with a range-dependent
                // aperture...
                // !!! beware, here (and in at least one other place), we are
                // copying the polar image data, but not deep-copying the
                // bearing/range data associated with the polar image
                NonUniformPolarMat r(a);
                r.mat = boost::get<cv::Mat>(operator()(a.mat));
                return r;
            }
            augmented_mat_t operator()(PyramidMat a) const{
                error() << "not implemented";
                return a;
            }
            const float m_sigma;
        };

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            float radius = param<float>("radius");

            augmented_mat_t img = inputs["image"]->augmentedMat();
            augmented_mat_t out = boost::apply_visitor(applyFastMedian(radius), img);

            r["image"] = boost::make_shared<Image>(out);

            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FAST_MEDIAN_NODE_H__
