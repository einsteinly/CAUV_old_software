#ifndef LOGVIEW_H
#define LOGVIEW_H

#include <QDockWidget>

#include <generated/messages_fwd.h>

#include "gui/cauvinterfaceelement.h"

namespace Ui {
    class LogView;
}

class QTextEdit;

namespace cauv {

    class AUV;

    class LogView : public QDockWidget, public CauvInterfaceElement {
        Q_OBJECT
    public:
        LogView(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        ~LogView();

    protected:
        virtual void initialise();
        virtual void appendLog(QTextEdit * edit, DebugType::e type, std::string message);

    private:
        Ui::LogView *ui;
    };
} // namespace
#endif // LOGVIEW_H
