#include <QtGui>

#include "viewWidget.h"

int main(int argc, char *argv[]){
    QApplication app(argc, argv);

    ViewWidget view;
    view.show();

    return app.exec();
}
