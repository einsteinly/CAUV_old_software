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

