#ifndef __SONAR_INPUT_NODE_H__
#define __SONAR_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>
#include <cstring>

#include <sonar/sonar_accumulator.h>
#include <utility/bash_cout.h>

#include <generated/types/SonarDataMessage.h>

#include "../node.h"


namespace cauv{
namespace imgproc{
                    

class SonarInputNode: public InputNode{
    public:
        SonarInputNode(ConstructArgs const& args)
            : InputNode(args),
              m_images_displayed(0),
              processed(0), m_counters_lock(),
              m_sonardata_msgs(), m_sonardata_lock() {
        }

        void init(){
            // don't want to drop lines:
            m_priority = priority_fastest;
            
            // registerInputID()
            registerParamID<int>("non-maximum suppression", 0, "0 for no suppression, 1 for only local maxima, 2 for only global maximum");
            registerParamID<int>("non-maximum epsilon", 1, "minimum difference for a node to be maximal");
            registerParamID<int>("low-pass width", 0, "radius of a 1D low pass filter applied to the data line");
            registerParamID<int>("min range", 0, "minimum range of bins drawn");
            registerParamID<int>("derivative", 0, "use the nth derivative of the data line (negative values reverse the direction)");

            // three output images, three output parameters:
            registerOutputID<image_ptr_t>("image (buffer)");
            registerOutputID<image_ptr_t>("image (synced)");
            registerOutputID<image_ptr_t>("data line");
            registerOutputID<NodeParamValue>("bearing");
            registerOutputID<NodeParamValue>("bearing range");
            registerOutputID<NodeParamValue>("range");
        }
    
        virtual ~SonarInputNode(){
            stop();
            info() << "~SonarInputNode statistics"
                   << "\n\tprocessed" <<  processed
                   << "\n\twaiting" << m_sonardata_msgs.size();
        }
        
        /**
         * ...
         */
        void onSonarDataMessage(boost::shared_ptr<const SonarDataMessage> m){
            lock_t l(m_sonardata_lock);
            debug(4) << "Input node received sonar data";
            
            if (numChildren() > 0)
            {
                m_sonardata_msgs.push_back(m);
                setAllowQueue();
            }
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            
            debug(4) << "SonarInputNode::doWork";
            
            std::vector< boost::shared_ptr<SonarDataMessage const> > msgs = latestSonarDataMessages();
            int nonMax = param<int>("non-maximum suppression");
            int nonMaxEpsilon = param<int>("non-maximum epsilon");
            int lowPassRadius = param<int>("low-pass width");
            int minRange = param<int>("min range");
            int derivative = param<int>("derivative");

            std::vector<float> halfKernel;
            float halfKernelSum = 0;
            if (lowPassRadius != 0) {
                float sigma = lowPassRadius/3.0f;
                for (int i = 0; i <= lowPassRadius; ++i) {
                    float val = std::exp(-(i*i)/(2*sigma*sigma));
                    if (i == 0)
                        halfKernelSum += val;
                    else
                        halfKernelSum += 2*val;
                    halfKernel.push_back(val);
                }
            }

            cv::Mat fullImage;
            foreach( boost::shared_ptr<SonarDataMessage const> m, msgs) {
                SonarDataLine l = m->line();
                std::vector<unsigned char>& bins = l.data;

                // Cut off bins before min range
                size_t minRangeBin = bins.size() * minRange / l.range;
                if (minRangeBin < bins.size()) {
                    for (size_t i = 0; i < minRangeBin && i < bins.size(); ++i)
                        bins[i] = bins[minRangeBin];
                } else {
                    for (size_t i = 0; i < minRangeBin && i < bins.size(); ++i)
                        bins[i] = 0;
                }
                
                // Low-pass the data
                if (lowPassRadius != 0) {
                    std::vector<unsigned char> lpbins(l.data.size(), 0);
                    for (size_t i = 0; i < bins.size(); ++i)
                        for (int k = 0; k <= lowPassRadius; ++k) {
                            if (k == 0)
                                lpbins[i] += halfKernel[0] * _get(bins,i) / halfKernelSum;
                            else
                                lpbins[i] += halfKernel[k] * (_get(bins,i-k) + _get(bins,i+k)) / halfKernelSum;
                        }
                    std::swap(lpbins, bins);
                }
                
                int d_sign = derivative > 0 ? 1 : -1;
                for (int d = 0; d < std::abs(derivative); ++d) {
                    std::vector<unsigned char> dbins(l.data.size(), 0);
                    for (size_t i = 0; i < bins.size(); ++i)
                        dbins[i] = clamp_cast<unsigned char>(d_sign*((int)_get(bins,i) - (int)_get(bins,i+1))+127);
                    std::swap(dbins, bins);
                }

                // Non-maximum suppression (drop anything that's not a local/global
                // maximum to zero)
                switch(nonMax) {
                    case 1: {
                        // Find local maxima
                        std::vector<unsigned char> lmbins(l.data.size(), 0);
                        for (int i = 1; i < (int)bins.size() - 1; ++i) {
                            if ((i-1 >= 0 && (bins[i-1] + nonMaxEpsilon > bins[i])) || (i+1 < (int)bins.size() && (bins[i] < bins[i+1] + nonMaxEpsilon))) {
                                lmbins[i] = 0;
                            } else {
                                lmbins[i] = bins[i];
                            }
                        }
                        std::swap(lmbins, bins);
                        break;
                    }
                    case 2: {
                        // Find global maximum
                        size_t imax = 0;
                        for (size_t i = 1; i < bins.size(); ++i) {
                            if (bins[i] < bins[imax] + nonMaxEpsilon) {
                                bins[i] = 0;
                            } else {
                                bins[imax] = 0;
                                imax = i;
                            }
                        }
                        break;
                    }
                }

                if (m_accumulator.accumulateDataLine(l))
                    fullImage = m_accumulator.mat().clone();
            }
            
            boost::shared_ptr<SonarDataMessage const> m_back = msgs.back();

            r["data line"] = boost::make_shared<Image>(cv::Mat(m_back->line().data, true).t());
            r["bearing"] = NodeParamValue(m_back->line().bearing);
            r["bearing range"] = NodeParamValue(m_back->line().bearingRange);
            r["range"] = NodeParamValue(m_back->line().range);
            
            // NB: output is not copied! use a CopyNode if you don't want to
            // stamp all over the buffer
            r["image (buffer)"] = m_accumulator.img();
            
            if (!fullImage.empty())
            {
                // deep copy:
                r["image (synced)"] = boost::make_shared<Image>(fullImage);
            }

            clearAllowQueue();

            return r;
        }

        std::vector< boost::shared_ptr<const SonarDataMessage> > latestSonarDataMessages(){
            lock_t l(m_sonardata_lock);
            debug(4) << "Grabbing sonar (" << m_sonardata_msgs.size() << " images)";
            
            std::vector< boost::shared_ptr<const SonarDataMessage> > ret;
            ret.swap(m_sonardata_msgs);
            processed++;
            debug(4) << "Processed" << processed << "images";
            return ret;
        }
        
        unsigned int _get(std::vector<unsigned char> const& data, int i)
        {
            return data[clamp_cast<size_t>(0, i, (int)data.size()-1)];
        }

    private:
        int m_images_displayed;
        SonarAccumulator m_accumulator;
    
        int processed;
        mutable boost::recursive_mutex m_counters_lock;
        
        std::vector < boost::shared_ptr<const SonarDataMessage> > m_sonardata_msgs;
        mutable boost::recursive_mutex m_sonardata_lock;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SONAR_INPUT_NODE_H__

