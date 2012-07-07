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

#ifndef __VIDEO_FILE_OUTPUT_NODE_H__
#define __VIDEO_FILE_OUTPUT_NODE_H__

#include <map>
#include <vector>
#include <string>
#include <cstdio>

#include <boost/bind.hpp>
#include <boost/algorithm/string/replace.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "outputNode.h"


namespace cauv{
namespace imgproc{

class VideoFileOutputNode: public OutputNode{
        typedef boost::mutex mutex_t;
        typedef boost::unique_lock<mutex_t> lock_t;
    public:
        VideoFileOutputNode(ConstructArgs const& args)
            : OutputNode(args), m_write_mux(), m_writer(), m_fname(), m_videoEmpty(true), m_counter(0){
        }

        void init(){
            // one input:
            registerInputID("image", true);

            // no outputs

            // parameters: the filename, jpg compression, png compression
            registerParamID<std::string>("filename", "out.%d.%t.%c.avi");
        }

        virtual ~VideoFileOutputNode(){
            lock_t l(m_write_mux);
            closeVideo();
        }

        virtual void paramChanged(input_id const& p){
            debug(4) << "VideoFileOutputNode::paramChanged";
            if(p == input_id("filename")){
                lock_t l(m_write_mux);
                closeVideo();
                setAllowQueue();
            }
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t&){
            image_ptr_t img = inputs["image"];

            debug(4) << "VideoFileOutputNode::doWork()" << inputs["image"];

            img->apply(boost::bind(&VideoFileOutputNode::processFrame, this, _1, param<std::string>("filename")));
        }

        void processFrame(cv::Mat& img, const std::string& filename)
        {
            using boost::algorithm::replace_all_copy;
            
            lock_t l(m_write_mux);
            try{
                if(!m_writer.isOpened())
                {
                    // Open a new video file with the given file name
                    // replace %d %t and %c with the current date, time and a counter
                    std::string cs = MakeString() << std::setfill('0') << std::setw(5) << m_counter++;
                    m_fname = replace_all_copy(
                                            replace_all_copy(
                                                replace_all_copy(filename, "%c", cs),
                                                    "%d", now("%Y-%m-%d")
                                            ), "%t", now("%H-%M-%s")
                                        );
                    openVideo(img.size());
                }

                accumulateFrame(img);
            }catch(cv::Exception& e){
                error() << "VideoFileOutputNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
                closeVideo();
                throw img_pipeline_error("VideoFileOutputNode: could not write file");
            }
        }

        void openVideo(cv::Size const& size){
            debug() << "VideoFileOutputNode::openVideo()" << m_fname;
            // TODO: automagical FPS detection
            bool r = m_writer.open(m_fname, CV_FOURCC('M','P','G','2'), 25 /*fps*/, size, true);
            m_videoEmpty = true;
            if(!r || !m_writer.isOpened()){
                error() << "could not open video writier";
                clearAllowQueue();
            }
        }

        void accumulateFrame(cv::Mat image){
            if(!m_writer.isOpened()){
                error() << "video writer is not open";
                return;
            }
            m_writer << image;
            m_videoEmpty = false;
        }

        void closeVideo(){
            if(m_writer.isOpened()){
                try{
                    m_writer = cv::VideoWriter();
                }catch(cv::Exception& e){
                    error() << "VideoFileOutputNode::closeVideo:\n\t"
                            << e.err << "\n\t"
                            << "in" << e.func << "," << e.file << ":" << e.line;
                }
                // Delete the file if nothing was written
                if (m_videoEmpty)
                {
                    // Could use boost::filesystem here, but opencv most likely
                    // uses cstdio, so may as well stick with it

                    debug() << "VideoFileOutputNode::closeVideo() : No frames written, deleting file " << m_fname;
                    remove(m_fname.c_str()); // Has a return code, but we ignore it
                }
            }
        }

        mutex_t m_write_mux;
        cv::VideoWriter m_writer;
        std::string m_fname;
        bool m_videoEmpty;
        int m_counter;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __VIDEO_FILE_OUTPUT_NODE_H__
