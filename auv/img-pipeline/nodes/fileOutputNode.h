#ifndef __FILE_OUTPUT_NODE_H__
#define __FILE_OUTPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/algorithm/string/replace.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "outputNode.h"


namespace cauv{
namespace imgproc{

class FileOutputNode: public OutputNode{
    public:
        FileOutputNode(ConstructArgs const& args)
            : OutputNode(args), m_counter(0){
        }

        void init(){
            // one input:
            registerInputID("image_in");
            
            // no outputs
            // registerOutputID<image_ptr_t>();
            
            // parameters: the filename, jpg compression, png compression
            registerParamID<std::string>("filename", "out.%d.%t.%c.jpg");
            registerParamID<int>("jpeg quality", 95); // 0-100
            registerParamID<int>("png compression", 9); // 0-9
        }
    
        virtual ~FileOutputNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            using boost::algorithm::replace_all_copy;
            out_map_t r;

            image_ptr_t img = inputs["image_in"];
            
            // replace %d %t and %c with the current date, time and a counter
            std::string cs = MakeString() << std::setfill('0') << std::setw(5) << m_counter++;
            std::string fname = replace_all_copy(
                                    replace_all_copy(
                                        replace_all_copy(
                                            param<std::string>("filename"), "%c", cs
                                        ), "%d", now("%Y-%m-%d")
                                    ), "%t", now("%H-%M-%s")
                                );
            int jpg_qual = param<int>("jpeg quality");
            int png_comp = param<int>("png compression");
             
            debug(4) << "FileOutputNode::doWork()" << *img << "->" << fname;

            std::vector<int> imwrite_params;
            imwrite_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            imwrite_params.push_back(jpg_qual);
            imwrite_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            imwrite_params.push_back(png_comp);
            
            try{
                cv::Mat in = img->mat();
                if(!in.empty())
                    cv::imwrite(fname.c_str(), in, imwrite_params);
                else
                    error() << "FileoutputNode: no image!";
            }catch(cv::Exception& e){
                error() << "FileOutputNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
                throw(img_pipeline_error("FileOutputNode: could not write file"));
            }
            
            return r;
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FILE_OUTPUT_NODE_H__
