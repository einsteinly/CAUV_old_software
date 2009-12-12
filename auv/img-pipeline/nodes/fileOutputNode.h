#ifndef __FILE_OUTPUT_NODE_H__
#define __FILE_OUTPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "../node.h"
#include "../image.h"


class FileOutputNode: public Node{
    public:
        FileOutputNode(Scheduler& s)
            : Node(s){
            // one input:
            registerInputID("image_in");
            
            // no outputs
            // registerOutputID();
            
            // parameters: the filename, jpg compression, png compression
            registerParamID<std::string>("filename", "default.out.jpg");
            registerParamID<int>("jpeg quality", 95); // 0-100
            registerParamID<int>("png compression", 9); // 0-9
        }

    protected:
        out_image_map_t doWork(in_image_map_t& inputs){
            out_image_map_t r;

            std::cerr << "fileOutputNode::doWork" << std::endl;
            
            image_ptr_t img = inputs["image_in"];
            std::string fname = param<std::string>("filename");
            int jpg_qual = param<int>("jpeg quality");
            int png_comp = param<int>("png compression");
               
            std::vector<int> imwrite_params;
            imwrite_params.push_back(CV_IMWRITE_JPEG_QUALITY);
            imwrite_params.push_back(jpg_qual);
            imwrite_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            imwrite_params.push_back(png_comp);

            cv::imwrite(fname.c_str(), img->cvMat(), imwrite_params);
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

// Register this node type
DEFINE_NFR(FileOutputNode, nt_file_output);

#endif // ndef __FILE_OUTPUT_NODE_H__
