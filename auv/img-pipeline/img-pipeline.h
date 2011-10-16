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

#ifndef __IMAGE_PIPELINE_H__
#define __IMAGE_PIPELINE_H__

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>

namespace cauv{
namespace imgproc{

class ImageProcessor;

class ImagePipelineNode : public CauvNode
{
    public:
        ImagePipelineNode();
    
    protected:
        virtual void onRun();
        virtual void addOptions(boost::program_options::options_description& desc,
                                boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm,
                                  boost::program_options::options_description& desc);

    private:
        std::string m_pipeline_name;
        boost::shared_ptr<ImageProcessor> m_pipeline;
};

} // namespace imgproc
} // namespace cauv

#endif //__IMAGE_PIPELINE_H__

