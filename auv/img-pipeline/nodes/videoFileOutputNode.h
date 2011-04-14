#ifndef __VIDEO_FILE_OUTPUT_NODE_H__
#define __VIDEO_FILE_OUTPUT_NODE_H__

#include <map>
#include <vector>
#include <string>
#include <cstdio>

#include <boost/algorithm/string/replace.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "outputNode.h"


namespace cauv{
namespace imgproc{

class VideoFileOutputNode: public OutputNode{
    public:
        VideoFileOutputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : OutputNode(sched, pl, t), m_writer(), m_counter(0){
        }

        void init(){
            // one input:
            registerInputID("image");
            
            // no outputs
            // registerOutputID<image_ptr_t>();
            
            // parameters: the filename, jpg compression, png compression
            registerParamID<std::string>("filename", "out.%d.%t.%c.avi");
        }

        virtual ~VideoFileOutputNode(){
            stop();
            closeVideo();
        }

        virtual void paramChanged(param_id const& p){
            debug(4) << "VideoFileOutputNode::paramChanged";
            if(p == param_id("filename")){
                closeVideo();
                setAllowQueue();
            }
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            using boost::algorithm::replace_all_copy;
            out_map_t r;

            image_ptr_t img = inputs["image"];
             
            debug(4) << "VideoFileOutputNode::doWork()" << *img;
            
            try{
                if(!m_writer.isOpened())
                {
                    // Open a new video file with the given file name
                    // replace %d %t and %c with the current date, time and a counter
                    std::string cs = MakeString() << std::setfill('0') << std::setw(5) << m_counter++;
                    m_fname = replace_all_copy(
                                            replace_all_copy(
                                                replace_all_copy(
                                                    param<std::string>("filename"), "%c", cs
                                                ), "%d", now("%Y-%m-%d")
                                            ), "%t", now("%H-%M-%s")
                                        );
                    openVideo(img->cvMat().size());
                }
                
                accumulateFrame(img);
            }catch(cv::Exception& e){
                error() << "VideoFileOutputNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
                closeVideo(); 
                throw img_pipeline_error("VideoFileOutputNode: could not write file");
            }
            
            return r;
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

        void accumulateFrame(image_ptr_t image){
            if(!m_writer.isOpened()){
                error() << "video writer is not open";
                return;
            }
            m_writer << image->cvMat();
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
