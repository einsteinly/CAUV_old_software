#ifndef CONSOLE_H
#define CONSOLE_H

#include <QDockWidget>

#include "cauvinterfaceelement.h"

namespace Ui {
    class Console;
}

class QConsole;

namespace cauv {

    class AUV;

    class Console : public QDockWidget, public CauvInterfaceElement {
        Q_OBJECT
    public:
        Console(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        ~Console();

    protected:
        virtual void initialise();
        QConsole * m_console;

    private:
        Ui::Console *ui;
    };

} //namespace cauv

#endif // CONSOLE_H
