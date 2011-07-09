#ifndef LOGVIEW_H
#define LOGVIEW_H

#include <QDockWidget>

#include <generated/types/DebugType.h>

#include <gui/core/cauvbasicplugin.h>

namespace Ui {
    class LogView;
}

class QTextEdit;

namespace cauv {

    class AUV;

    class LogView : public QDockWidget, public CauvBasicPlugin {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)

    public:
        LogView();
        ~LogView();

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;
        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node);

    protected:
        virtual void appendLog(QTextEdit * edit, DebugType::e type, std::string message);

    private:
        Ui::LogView *ui;
    };
} // namespace
#endif // LOGVIEW_H
