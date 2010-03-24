#include <QApplication>

#include "PipelineWidget.h"
 
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    PipelineWidget w;
    
    w.show();
    return app.exec();
}
