#ifndef __SONAR_INPUT_NODE_H__
#define __SONAR_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>
#include <cstring>

#include <sonar/sonar_accumulator.h>
#include <utility/bash_cout.h>

#include <generated/types/SonarDataMessage.h>
#include <generated/types/SonarImageMessage.h>

#include "../node.h"


namespace cauv{
namespace imgproc{
                    

class SonarInputNode: public InputNode{
    public:
        SonarInputNode(ConstructArgs const& args)
            : InputNode(args),
              m_sonar_id(SonarID::Seasprite),
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
            registerParamID<int>("Sonar ID", SonarID::Seasprite, "SonarID::Seasprite=1, SonarID::Gemini=2");
            registerParamID<int>("Resolution", 400, "SonarAccumulator resolution");

            // four output images, three output parameters:
            registerOutputID<image_ptr_t>("image (buffer)");
            registerOutputID<image_ptr_t>("image (synced)");
            registerOutputID<image_ptr_t>("polar image");
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
        virtual void onSonarDataMessage(boost::shared_ptr<const SonarDataMessage> m){
            // backwards compatibilty: only the seasprite sends data one
            // line at a time:
            if(m_sonar_id != SonarID::Seasprite)
                return;

            lock_t l(m_sonardata_lock);
            debug(4) << "Input node received sonar data";
            
            if (numChildren() > 0)
            {
                m_sonardata_msgs.push_back(m);
                setAllowQueue();
            }
        }

        virtual void onSonarImageMessage(boost::shared_ptr<const SonarImageMessage> m){
            if(m_sonar_id != m->source())
                return;

            lock_t l(m_sonardata_lock);
            // maybe dropping an old and unprocessed message here
            m_sonarimg_msg = m;
            setAllowQueue();
        }

        virtual void paramChanged(input_id const& param_id){
            if(param_id == "Sonar ID")
                m_sonar_id = SonarID::e(param<int>("Sonar ID"));
        }

    protected:
        out_map_t doWork_dataLines(){
            out_map_t r;
            
            debug(4) << "SonarInputNode::doWork_dataLines";
            
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

            return r;
        }
        
        // see comment below, where this structure is used
        struct MessageImageDeleter{
            MessageImageDeleter(boost::shared_ptr<const Message> m) : m_msg(m){ }
            void operator()(Image* img){
                delete img;
                m_msg.reset();
                // now m_msg may be deleted (depending on other outstanding
                // shared pointers)
            }
            boost::shared_ptr<const Message> m_msg;
        };
        out_map_t doWork_image(){
            out_map_t r;
            boost::shared_ptr<SonarImageMessage const> image_msg;            
            
            debug(4) << "SonarInputNode::doWork_image";

            {  lock_t l(m_sonardata_lock);
                image_msg.swap(m_sonarimg_msg);
            }

            if(m_accumulator.setWholeImage(image_msg->image()))
                // not a deep copy when we're accumulating whole images!
                r["image (synced)"] = m_accumulator.img();

            NonUniformPolarMat r_polar_mat;
            uint32_t rows = image_msg->image().rangeEnd - image_msg->image().rangeStart;
            uint32_t cols = image_msg->image().bearing_bins.size();
            // actually one less than bearing bins.size:
            if(cols) cols--;
            r_polar_mat.mat = cv::Mat(rows, cols, CV_8UC3, (void*) &(image_msg->image().data[0]));
            r_polar_mat.bearings = boost::make_shared< std::vector<float> >(cols);
            r_polar_mat.ranges = boost::make_shared< std::vector<float> >(rows);
            std::vector<int32_t> const& bearing_bins = image_msg->image().bearing_bins;
            for(int i = 0; i < int(bearing_bins.size())-1; i++){
                // bearing_bins has the edge angles - not the centre angles:
                // interpolate linearly (close enough)
                int32_t a = bearing_bins[i];
                int32_t b = bearing_bins[i+1];
                (*r_polar_mat.bearings)[i] = (
                    (msgPolarAngleToRadians(a) + msgPolarAngleToRadians(b)) / 2
                );
            }
            float range_convert = image_msg->image().rangeConversion;
            float range = image_msg->image().rangeStart;
            for(int i = 0; i < r_polar_mat.mat.rows; i++){
                range += range_convert;
                (*r_polar_mat.ranges)[i] = range;
            }
            
            // so here's some magic: avoid copying by pointing the polar image
            // to the data that's in the message we received: this is a shared
            // pointer, (as are images), so keep it alive for as long as the
            // returned image exists by using a deleter that holds a copy
            image_ptr_t r_polar_img = boost::shared_ptr<Image>(
                new Image(r_polar_mat), MessageImageDeleter(image_msg)
            );
            r_polar_img->ts(image_msg->image().timeStamp);
            r["polar image"] = r_polar_img;
                
            // !!! TODO: probably want to set the single-line output to the
            // line pointing straight forwards, or something similar
            return r;
        }

        out_map_t doWork(in_image_map_t&){
            clearAllowQueue();
            int resolution = param<int>("Resolution");
            m_accumulator.setSize(resolution);
            
            if(m_sonarimg_msg || m_sonar_id == SonarID::Gemini){
                // in this case discard any SonarDataMessages that have been
                // received, to prevent indefinite build-up if for some crazy
                // reason they are also being sent
                {   lock_t l(m_sonardata_lock);
                    m_sonardata_msgs.clear();
                }
                return doWork_image();
            }else{
                return doWork_dataLines();
            }
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
        
        static unsigned int _get(std::vector<unsigned char> const& data, int i)
        {
            return data[clamp_cast<size_t>(0, i, (int)data.size()-1)];
        }

    private:
        SonarID::e m_sonar_id;

        int m_images_displayed;
        SonarAccumulator m_accumulator;
    
        int processed;
        mutable boost::recursive_mutex m_counters_lock;
        
        std::vector < boost::shared_ptr<const SonarDataMessage> > m_sonardata_msgs;
        boost::shared_ptr<const SonarImageMessage> m_sonarimg_msg;
        mutable boost::recursive_mutex m_sonardata_lock;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SONAR_INPUT_NODE_H__

