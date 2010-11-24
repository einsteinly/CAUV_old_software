#include "cauvgui.h"

#include <qwt_plot.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_legend.h>
#include <qwt_data.h>
#include <qwt_text.h>
#include <qwt_math.h>
#include <math.h>
#include "canvaspicker.h"
#include "scalepicker.h"
#include "plot.h"

extern QApplication* app;

CauvGui::CauvGui(QWidget *parent) : CauvNode("CauvGui"){
    setupUi(this);
    joinGroup("control");
}

void CauvGui::onRun()
{
    CauvNode::onRun();
    Plot* plot = new Plot();

    setCentralWidget(plot);

    (void) new CanvasPicker(plot);

    // The scale picker translates mouse clicks
    // int o clicked() signals

    ScalePicker *scalePicker = new ScalePicker(plot);
    plot->connect(scalePicker, SIGNAL(clicked(int, double)),
        plot, SLOT(insertCurve(int, double)));

    show();
    app->exec();
}
