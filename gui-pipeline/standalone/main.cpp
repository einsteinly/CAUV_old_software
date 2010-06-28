#include <QApplication>

#include "pipelineWidget.h"
 
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    pw::PipelineWidget *w = new pw::PipelineWidget(0, argc, argv);

    w->show();
    return app.exec();
}
