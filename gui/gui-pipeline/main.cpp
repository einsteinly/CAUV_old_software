#include <QApplication>

#include "pipelineWidget.h"
#include "pipelineWidgetNode.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    pw::PipelineWidget *w = new pw::PipelineWidget();

    // start the cauv node thread
    boost::thread(pw::spawnPGCN, w, argc, argv),

    w->show();
    return app.exec();
}
