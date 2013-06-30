/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef PROCESSSTATEVIEW_H
#define PROCESSSTATEVIEW_H

#include <QTableWidget>
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

    /* ProcessState specialization */
    template<>

    class DataStreamSplitter<cauv::ProcessState> : public DataStreamTool {

    public:
        DataStreamSplitter<cauv::ProcessState>(boost::shared_ptr<DataStream<cauv::ProcessState> > stream) :
                combined(stream),
                process(boost::make_shared<DataStream<std::string> >("process", "", stream.get())),
                status(boost::make_shared<DataStream<std::string> >("status", "", stream.get())),
                cpu(boost::make_shared<DataStream<float> >("cpu", "%", stream.get())),
                mem(boost::make_shared<DataStream<float> >("mem", "%", stream.get())),
                threads(boost::make_shared<DataStream<uint32_t> >("threads", "", stream.get()))
        {
            // getter function binds
            boost::function<std::string(cauv::ProcessState)> processGetter = boost::bind(&cauv::ProcessState::process, _1);
            boost::function<std::string(cauv::ProcessState)> statusGetter = boost::bind(&cauv::ProcessState::status, _1);
            boost::function<float(cauv::ProcessState)> cpuGetter = boost::bind(&cauv::ProcessState::cpu, _1);
            boost::function<float(cauv::ProcessState)> memGetter = boost::bind(&cauv::ProcessState::mem, _1);
            boost::function<uint32_t(cauv::ProcessState)> threadsGetter = boost::bind(&cauv::ProcessState::threads, _1);

            // connect up the slots
            // the getter functions are passed to the new data streams and evaluated there
            stream->onUpdate.connect(boost::bind(&DataStream<std::string>::update<cauv::ProcessState>, process.get(), processGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<std::string>::update<cauv::ProcessState>, status.get(), statusGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::ProcessState>, cpu.get(), cpuGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::ProcessState>, mem.get(), memGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<uint32_t>::update<cauv::ProcessState>, threads.get(), threadsGetter, _1));
        }

        boost::shared_ptr<DataStream<ProcessState> > combined;
        boost::shared_ptr<DataStream<std::string> > process;
        boost::shared_ptr<DataStream<std::string> > status;
        boost::shared_ptr<DataStream<float> > cpu;
        boost::shared_ptr<DataStream<float> > mem;
        boost::shared_ptr<DataStream<uint32_t> > threads;
    };



    class ProcessStateView : public QDockWidget, public CauvBasicPlugin {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)
    public:
        ProcessStateView();
        virtual ~ProcessStateView();

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;
        virtual void initialise(boost::shared_ptr<AUV>, boost::shared_ptr<CauvNode> node);
        virtual void onNewProcess(boost::shared_ptr<DataStream<ProcessState> > stream);

    Q_SIGNALS:
        void processStateUpdated(const std::string& process, const float cpu, const float mem, const float threads, const std::string& status);

    protected Q_SLOTS:
        void onProcessStateUpdate(const std::string& process, const float cpu, const float mem, const float threads, const std::string& status);

    protected:        
        void onProcessStateUpdate(const ProcessState &state);
        boost::unordered_map<std::string, int> m_processes;
        std::vector<boost::shared_ptr<DataStreamSplitter<ProcessState> > > m_splitters;

    private:
        Ui::ProcessStateView *ui;
    };

} // namesapce cauv



class DraggableTableWidget : public QTableWidget, public cauv::DataStreamDragSource {

public:

    DraggableTableWidget(QWidget * parent = 0) : QTableWidget(parent){}

    boost::shared_ptr<std::vector<boost::shared_ptr<cauv::DataStreamBase> > > getDataStreams();
};

#endif // PROCESSSTATEVIEW_H
