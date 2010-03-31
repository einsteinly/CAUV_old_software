#include <QApplication>

#include "pipelineWidget.h"
 
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    pw::PipelineWidget *w = new pw::PipelineWidget;

    w->show();
    return app.exec();
}
