/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __FILE_OUTPUT_NODE_H__
#define __FILE_OUTPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/algorithm/string/replace.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>

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
            registerInputID("image_in", Const);
            
            // no outputs
            // registerOutputID("...", default_vaue);
            
            // parameters: the filename, jpg compression, png compression
            registerParamID<std::string>("filename", "out.%d.%t.%c.jpg");
            registerParamID<int>("jpeg quality", 95); // 0-100
            registerParamID<int>("png compression", 9); // 0-9
        }

    protected:
        struct writeMat: boost::static_visitor<void>{
            writeMat(const std::string& fname, std::vector<int> const& imwrite_params)
                : m_fname(fname), m_imwrite_params(imwrite_params){
            }
            void operator()(cv::Mat a) const{
                cv::imwrite(m_fname.c_str(), a, m_imwrite_params);
            }
            void operator()(NonUniformPolarMat a) const{
                // !!! TODO: this throws away metadata which should somehow be
                // saved (sidecar file?)
                cv::imwrite(m_fname.c_str(), a.mat, m_imwrite_params);
            }
            void operator()(PyramidMat a) const{
                // !!! TODO: presuming 0 is the only level worth saving
                cv::imwrite(m_fname.c_str(), a.levels.at(0), m_imwrite_params);
            }
            const std::string& m_fname;
            std::vector<int> const& m_imwrite_params;
        };
        void doWork(in_image_map_t& inputs, out_map_t&){
            using boost::algorithm::replace_all_copy;

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
                augmented_mat_t t = img->augmentedMat();
                boost::apply_visitor(writeMat(fname, imwrite_params), t);
            }catch(cv::Exception& e){
                error() << "FileOutputNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
                throw(img_pipeline_error("FileOutputNode: could not write file"));
            }
            
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FILE_OUTPUT_NODE_H__
