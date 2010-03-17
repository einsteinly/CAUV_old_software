#ifndef __IMAGE_PIPELINE_H__
#define __IMAGE_PIPELINE_H__

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>

class ImageProcessor;

class ImagePipelineNode : public CauvNode
{
    public:
        ImagePipelineNode();
    
    protected:
        virtual void onRun();

    private:
        boost::shared_ptr<ImageProcessor> m_pipeline;
};

#endif //__IMAGE_PIPELINE_H__

