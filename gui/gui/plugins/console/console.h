#ifndef CONSOLE_H
#define CONSOLE_H

#include <boost/unordered_map.hpp>

#include <QDockWidget>

#include <model/auv_model.h>

#include "qconsole2/include/qconsole.h"

#include <gui/core/cauvbasicplugin.h>

namespace Ui {
    class Console;
}

namespace cauv {

    class CauvConsole : public QConsole {
        Q_OBJECT
    public:
        CauvConsole(const char * name, QWidget * parent = NULL);

        void execCommand(QString command, QString response, DebugType::e type, bool writeCommand, bool showPrompt);

        bool isCommandComplete(QString command);

    Q_SIGNALS:
        void commandReady(QString);
    };



    class Console : public QDockWidget, public CauvBasicPlugin {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)
    public:
        Console();
        ~Console();

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;
        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node);

    protected:
        CauvConsole * m_console;
        boost::unordered_map<int, QString > m_requests;
        unsigned int m_counter;

    protected Q_SLOTS:
        void executeCommand(QString s);
        void onResponse(int, QString response, DebugType::e level);
        void onResponse(ScriptResponse response);

    Q_SIGNALS:
        void responseReceived(int, QString, DebugType::e);

    private:
        Ui::Console *ui;
    };

} //namespace cauv

#endif // CONSOLE_H
