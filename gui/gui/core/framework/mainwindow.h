#ifndef CAUVGUI_H
#define CAUVGUI_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <boost/enable_shared_from_this.hpp>
#include <common/cauv_node.h>

#include "guiactions.h"

namespace Ui {
    class MainWindow;
}
class QDir;

namespace cauv {

    namespace gui {

        class CauvMainWindow : public QMainWindow, public CauvNode, public boost::enable_shared_from_this<CauvMainWindow> {
            Q_OBJECT

        public:
            CauvMainWindow(QApplication * app);
            virtual ~CauvMainWindow();

        public Q_SLOTS:
            int send(boost::shared_ptr<const Message>message);

        protected:
            virtual void onRun();
            virtual bool loadPlugin(QObject * plugin);
            virtual void closeEvent(QCloseEvent *);

            QApplication * m_application;
            boost::shared_ptr<GuiActions> m_actions;

        private:
            Ui::MainWindow * ui;
            int findPlugins(const QDir& dir, int subdirs = 0);

        };

    } //namespace gui
} // namespace cauv

#endif // CAUVGUI_H
