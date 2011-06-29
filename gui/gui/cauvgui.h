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

    namespace gui {

        class AUV;

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

            QApplication * m_application;

        private:
            Ui::MainWindow * ui;

            int findPlugins(const QDir& dir, int subdirs = 0);

        };

    } //namespace gui

} // namespace cauv

#endif // CAUVGUI_H
