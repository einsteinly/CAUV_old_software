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

#include <QApplication>

#include "pipelineWidget.h"
#include "pipelineWidgetNode.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    cauv::gui::pw::PipelineWidget *w = new cauv::gui::pw::PipelineWidget();

    // start the cauv node thread
    boost::thread(cauv::gui::pw::spawnPGCN, w, argc, argv),

    w->show();
    return app.exec();
}
