#ifndef CAUVGUI_H
#define CAUVGUI_H

#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <common/cauv_node.h>
#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

#include <model/auv_model.h>
#include <model/auv_controller.h>

#include "ui_mainwindow.h"
#include "pipelineWidget.h"


class CauvGui : public QMainWindow, public CauvNode, private Ui::MainWindow {

    Q_OBJECT

    public:
        CauvGui(QApplication& app, QWidget *parent = 0);

    public Q_SLOTS:
        int send(boost::shared_ptr<Message>message);

    protected:
        virtual void onRun();

        virtual void closeEvent(QCloseEvent *);

        boost::shared_ptr<AUV> m_auv;
        boost::shared_ptr<AUVController> m_auv_controller;

        QApplication &m_application;
};

#endif // CAUVGUI_H
