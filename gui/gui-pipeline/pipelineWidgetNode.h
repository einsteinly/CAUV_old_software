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

#ifndef PIPELINEWIDGETNODE_H
#define PIPELINEWIDGETNODE_H

#include <common/cauv_node.h>
#include <generated/types/message.h>

#include <QObject>

#include "pipelineWidget.h"

namespace cauv {
namespace gui {
namespace pw {

class PipelineGuiCauvNode: public QObject, public CauvNode {
    Q_OBJECT
    public:
        PipelineGuiCauvNode(PipelineWidget *p);
        void onRun();

    public Q_SLOTS:
        int send(boost::shared_ptr<Message> message);

    private:
        PipelineWidget *m_widget;
};

// creating threads taking parameters (especially in a ctor-initializer) is a
// little tricky, using an intermediate function smooths the ride a bit:
void spawnPGCN(PipelineWidget *p, int argc, char** argv);

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // PIPELINEWIDGETNODE_H
