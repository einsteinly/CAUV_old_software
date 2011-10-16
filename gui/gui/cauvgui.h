/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef CAUVGUI_H
#define CAUVGUI_H

#include <QMainWindow>

#include <boost/enable_shared_from_this.hpp>

#include <common/cauv_node.h>


namespace Ui {
    class MainWindow;
}
class QDir;

namespace cauv {

    class AUV;
    class AUVController;

    class CauvGui : public QMainWindow, public CauvNode, public boost::enable_shared_from_this<CauvGui> {
        Q_OBJECT

    public:
        CauvGui(QApplication * app);
        virtual ~CauvGui();

    public Q_SLOTS:
        int send(boost::shared_ptr<Message>message);
        void addCentralTab(QWidget* tab, const QString& name);
        void addCentralTab(QWidget* tab, QString& name);
        void addDock(QDockWidget* dock, Qt::DockWidgetArea area);

    protected:
        virtual void onRun();

        virtual bool loadPlugin(QObject * plugin);

        virtual void closeEvent(QCloseEvent *);

        boost::shared_ptr<AUV> m_auv;
        boost::shared_ptr<AUVController> m_auv_controller;

        QApplication * m_application;

    private:
        Ui::MainWindow * ui;

        int findPlugins(const QDir& dir, int subdirs = 0);

    };

} // namespace cauv

#endif // CAUVGUI_H
