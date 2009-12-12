#ifndef __IMAGE_PIPELINE_H__
#define __IMAGE_PIPELINE_H__

#include <common/cauv_node.h>

class ImagePipelineNode : public CauvNode
{
    public:
        ImagePipelineNode(const std::string& group);
        virtual ~ImagePipelineNode();
    
    protected:
        virtual void onRun();
};

#endif //__IMAGE_PIPELINE_H__

