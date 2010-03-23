#include <QMainWindow>

#include "ui_standalone.h"
 
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QMainWindow mw;
    Ui_standalone ui;
    ui.setupUi(&mw);
    
    mw.show();
    return app.exec();
}
