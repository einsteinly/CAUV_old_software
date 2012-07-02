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

#ifndef __FILE_INPUT_NODE_H__
#define __FILE_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "asynchronousNode.h"


namespace cauv{
namespace imgproc{

class FileInputNode: public AsynchronousNode{
        typedef boost::unique_lock<boost::recursive_mutex> lock_t;
    public:
        FileInputNode(ConstructArgs const& args)
            : AsynchronousNode(args),
              m_is_directory(false), m_iter(), m_seq(0),
              m_instance_num(nextInstanceNum()){
        }

        void init(){
            // no inputs
            // registerInputID()
            
            // one output:
            registerOutputID("image");
            
            // one parameter: the filename
            registerParamID<std::string>("filename", "default.jpg");
        }

        virtual void paramChanged(input_id const& p){
            debug(4) << "FileInputNode::paramChanged";
            if(p == input_id("filename")){
                lock_t l(m_dir_mutex);
                std::string fname = param<std::string>("filename");
                if(boost::filesystem::is_directory(fname)){
                    m_is_directory = true;
                    closeVideo();
                }else{
                    m_is_directory = false;
                    openVideo(fname);
                }
                m_iter = boost::filesystem::directory_iterator();
                setAllowQueue();
            }else{
                warning() << "unknown parameter" << p << "set";
            }
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            debug(4) << "fileInputNode::doWork";
        
            std::string fname = param<std::string>("filename");
            cv::Mat image;

            if(!m_is_directory){
                lock_t cl(m_capture_lock);
                if(!m_capture.isOpened()){
                    clearAllowQueue();
                }else{
                    m_capture >> image;
                    if(image.empty()){
                        debug() << "video stream seems to have finished";
                        closeVideo();
                        clearAllowQueue();
                    }
                }
            }else{
                lock_t l(m_dir_mutex);
                const boost::filesystem::directory_iterator end;
                if(m_iter == end)
                    m_iter = boost::filesystem::directory_iterator(fname);
                // continue until the directory is exhausted, or an image is
                // successfully loaded
                for(; m_iter != end; m_iter++){
                    debug(4)  << "considering path:" << m_iter->path().native();
                    if(!boost::filesystem::is_directory(m_iter->status())) {
                        image = cv::imread(m_iter->path().native());
                        if (!image.empty())
                        {
                            m_iter++;
                            break;
                        }
                    }
                }
                if(image.empty())
                    warning() << "no images in directory" << fname;
                // NB: allowQueue not cleared
            }
            if(image.empty() || image.rows == 0 || image.cols == 0)
                throw user_attention_error("invalid image:" + fname);
            r.internalValue("image") = boost::make_shared<Image>(image, now(), mkUID(SensorUIDBase::File + m_instance_num, ++m_seq));
        }

        bool openVideo(std::string const& fname){
            lock_t l(m_capture_lock);
            if(m_capture.open(fname))
                return true;
            else
                m_capture.release();
            return false;
        }

        void closeVideo(){
            m_capture.release();
        }

    private:
        static uint32_t nextInstanceNum(){
            // node creation actually always happens on the same thread, so we
            // don't need a lock here
            static uint32_t instance_num = 0;
            instance_num++;
            // if there are more than 32 fileinput nodes, this will be a
            // problem:
            return instance_num % 0x20;
        }

        boost::recursive_mutex m_dir_mutex;

        bool m_is_directory;
        boost::filesystem::directory_iterator m_iter;

        cv::VideoCapture m_capture;
        boost::recursive_mutex m_capture_lock;

        uint64_t m_seq;
        uint32_t m_instance_num;
    
        // Register this node type
        DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FILE_INPUT_NODE_H__

