#ifndef PROCESSSTATEVIEW_H
#define PROCESSSTATEVIEW_H

#include <QDockWidget>

#include <boost/make_shared.hpp>
#include <boost/unordered_map.hpp>

#include <gui/core/cauvbasicplugin.h>

#include <gui/core/datastreamdragging.h>

#include <model/auv_model.h>

namespace Ui {
    class ProcessStateView;
}

namespace cauv {

    class ProcessStateView : public QDockWidget, public CauvBasicPlugin, public cauv::DataStreamDragSource {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)
    public:
        ProcessStateView();
        virtual ~ProcessStateView();

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;
        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node);
        boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > getDataStreams() const;

    Q_SIGNALS:
        void processStateUpdated(const std::string process, const float cpu, const float mem, const float threads, std::string status);

    protected Q_SLOTS:
        void onProcessStateUpdate(const std::string process, const float cpu, const float mem, const float threads, std::string status);

    protected:        
        void onProcessStateUpdate(const ProcessState &state);
        boost::unordered_map<std::string, int> m_processes;

    private:
        Ui::ProcessStateView *ui;
    };

} // namesapce cauv

#endif // PROCESSSTATEVIEW_H
