#ifndef __PIPELINE_WIDGET_NODE_H__
#define __PIPELINE_WIDGET_NODE_H__

#include <common/cauv_node.h>
#include <common/messages.h>

#include "pwTypes.h"

namespace pw{

struct DBGLevelObserver: MessageObserver
{
    void onDebugLevelMessage(DebugLevelMessage_ptr m);
};

class PipelineGuiMsgObs: public BufferedMessageObserver{
    public:
        PipelineGuiMsgObs(PipelineWidget *p);

        virtual void onNodeAddedMessage(NodeAddedMessage_ptr m);
        virtual void onNodeRemovedMessage(NodeRemovedMessage_ptr m);
        virtual void onNodeParametersMessage(NodeParametersMessage_ptr m);
        virtual void onArcAddedMessage(ArcAddedMessage_ptr m);
        virtual void onArcRemovedMessage(ArcRemovedMessage_ptr m);
        virtual void onGraphDescriptionMessage(GraphDescriptionMessage_ptr m);
        virtual void onStatusMessage(StatusMessage_ptr m);
        virtual void onInputStatusMessage(InputStatusMessage_ptr m);
        virtual void onOutpuStatusMessage(OutputStatusMessage_ptr m);
        virtual void onGuiImageMessageBuffered(GuiImageMessage_ptr m);

    private:
        PipelineWidget *m_widget;
};

class PipelineGuiCauvNode: public CauvNode{
    public:
        PipelineGuiCauvNode(PipelineWidget *p);
        void onRun();
    private:
        PipelineWidget *m_widget;
};

// creating threads taking parameters (especially in a ctor-initializer) is a
// little tricky, using an intermediate function smooths the ride a bit:
void spawnPGCN(PipelineWidget *p, int argc, char** argv);

} // namespace pw

#endif // ndef __PIPELINE_WIDGET_NODE_H__
