#include <QApplication>

#include "pipelineWidget.h"
#include "pipelineWidgetNode.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    cauv::pw::PipelineWidget *w = new cauv::pw::PipelineWidget();

    // start the cauv node thread
    boost::thread(cauv::pw::spawnPGCN, w, argc, argv),

    w->show();
    return app.exec();
}
