#ifndef PROCESSSTATEVIEW_H
#define PROCESSSTATEVIEW_H

#include <QWidget>
#include <QTableWidgetItem>

#include <boost/make_shared.hpp>
#include <boost/unordered_map.hpp>

#include <debug/cauv_debug.h>

#include "cauvinterfaceelement.h"

#include <model/auv_model.h>

namespace Ui {
    class ProcessStateView;
}

namespace cauv {

    class ProcessStateView : public QWidget, public CauvInterfaceElement {
        Q_OBJECT
    public:
        ProcessStateView(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual ~ProcessStateView();

        virtual void initialise();

    protected:
        void onProcessStateUpdate(const ProcessState &state);

    Q_SIGNALS:
        void processStateUpdated(const std::string process, const float cpu, const float mem, const float threads, std::string status);

    protected Q_SLOTS:
        void onProcessStateUpdate(const std::string process, const float cpu, const float mem, const float threads, std::string status);

    protected:
        boost::unordered_map<std::string, int> m_processes;

    private:
        Ui::ProcessStateView *ui;
    };

} // namesapce cauv

#endif // PROCESSSTATEVIEW_H
