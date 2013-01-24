/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
