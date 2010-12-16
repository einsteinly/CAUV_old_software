#ifndef __PIPELINE_MESSAGE_OBSERVER_H__
#define __PIPELINE_MESSAGE_OBSERVER_H__

#include <common/cauv_node.h>
#include <generated/messages.h>

#include <QObject>

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

} // namespace pw

#endif // ndef __PIPELINE_MESSAGE_OBSERVER_H__
