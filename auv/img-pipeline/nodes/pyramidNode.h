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

#ifndef __PYRAMID_NODE_H__
#define __PYRAMID_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"
#include "../nodeFactory.h"


//Used for cropping (self-explanatory) and anti-cropping (ask James Crosby)

namespace cauv{
namespace imgproc{

class PyramidNode: public Node{
    public:
        PyramidNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID(Image_In_Name, Const);
            
            // output:
            registerOutputID(Image_Out_Copied_Name);
            
            // multiple parameters
            registerParamID<int>("level",1,"positive level will call pyrdown, negative will call pyrup");
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
            try{
                int level = param<int>("level");
            
                cv::Mat img = inputs[Image_In_Name]->mat();

                cv::Mat out_mat = img;
                cv::Mat tmp;
                if (level < 0)
                {
                    for (int i = 0; i < -level; ++i)
                    {
                        cv::pyrUp(out_mat, tmp);
                        out_mat = tmp;
                    }
                }
                else
                {
                    for (int i = 0; i < level; ++i)
                    {
                        cv::pyrDown(out_mat, tmp);
                        out_mat = tmp;
                    }
                }
                
                r[Image_Out_Copied_Name] = boost::make_shared<Image>(out_mat);
        
            } catch(cv::Exception& e) {
                error() << "PyramidNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
        }
        
        // Register this node type
        DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __PYRAMID_NODE_H__

