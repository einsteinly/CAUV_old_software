/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_MAINWINDOW_H__
#define __CAUV_MAINWINDOW_H__

#include <QtGui>
#include <boost/enable_shared_from_this.hpp>
#include <common/cauv_node.h>

class QDir;

namespace Ui {
class MainWindow;
}

namespace cauv {

class MessageObserver;

namespace gui {


class CauvInterfacePlugin;
struct GuiActions;

class StackWidget: public QWidget{
    Q_OBJECT
    public:
        StackWidget(QWidget* parent = 0);

    public Q_SLOTS:
        void push(QString name, QWidget* widget);
        void pop();
    
    private:
        void updateTitle();

        QLabel* m_title; // replace with something snazzier...
        QStackedWidget* m_stack_widget;

        QStack<QPair<QString, QWidget*> > m_stack;

        QPropertyAnimation * m_titleAnimation;
};

class CauvMainWindow : public QMainWindow, public CauvNode, public boost::enable_shared_from_this<CauvMainWindow> {
    Q_OBJECT

public:
    CauvMainWindow(QApplication * app);
    virtual ~CauvMainWindow();

    StackWidget* viewStack();

public Q_SLOTS:
    int send(boost::shared_ptr<const Message>message);
    void registerObserver(boost::shared_ptr<MessageObserver>observer);
    void unregisterObserver(boost::shared_ptr<MessageObserver>observer);
    void createContextMenu(QPoint point);

protected:
    virtual void onRun();
    virtual CauvInterfacePlugin * loadPlugin(QObject * plugin);
    virtual void closeEvent(QCloseEvent *);

    QApplication * m_application;
    // actions are the way the GUI exposes itself and the vehicles model to plugins
    boost::shared_ptr<GuiActions> m_actions;
    std::vector<CauvInterfacePlugin *> m_plugins;

private:
    Ui::MainWindow * ui;
    int findPlugins(const QDir& dir, int subdirs = 0);

    StackWidget* m_view_stack;
};

} //namespace gui
} // namespace cauv

#endif // __CAUV_MAINWINDOW_H__
