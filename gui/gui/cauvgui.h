#ifndef CAUVGUI_H
#define CAUVGUI_H

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <common/cauv_node.h>
#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

#include "ui_mainwindow.h"
#include "pipelineWidget.h"


class CauvGui : public QMainWindow, public CauvNode, private Ui::MainWindow {
    public:
        CauvGui(QWidget *parent = 0);

    protected:
        virtual void onRun();
};

#endif // CAUVGUI_H
