#ifndef CONSOLE_H
#define CONSOLE_H

#include <boost/unordered_map.hpp>

#include <QDockWidget>

#include <model/auv_model.h>

#include "cauvinterfaceelement.h"
#include "qconsole2/include/qconsole.h"

namespace Ui {
    class Console;
}

namespace cauv {

    class CauvConsole : public QConsole {
        Q_OBJECT
    public:
        CauvConsole(const char * name, QWidget * parent = NULL);

        void execCommand(QString command, QString response, bool writeCommand, bool showPrompt);

        bool isCommandComplete(QString command);

    Q_SIGNALS:
        void commandReady(QString);
    };



    class Console : public QDockWidget, public CauvInterfaceElement {
        Q_OBJECT
    public:
        Console(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        ~Console();

    protected:
        virtual void initialise();
        CauvConsole * m_console;

        boost::unordered_map<int, QString > m_requests;

        unsigned int m_counter;

    protected Q_SLOTS:
        void executeCommand(QString s);

        void onResponse(int, QString response);
        void onResponse(script_exec_response_t response);

    Q_SIGNALS:
        void responseReceived(int, QString);


    private:
        Ui::Console *ui;
    };

} //namespace cauv

#endif // CONSOLE_H
