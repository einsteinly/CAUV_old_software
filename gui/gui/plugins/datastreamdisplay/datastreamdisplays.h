#ifndef DATASTREAMDISPLAYS_H
#define DATASTREAMDISPLAYS_H

#include <QDockWidget>
#include <QVariant>
#include <QTreeWidget>
#include <QMdiArea>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>
#include <model/auv_model.h>

#include <gui/core/cauvbasicplugin.h>
#include <gui/core/datastreamdragging.h>

namespace Ui {
    class DataStreamPicker;
}


namespace cauv {

    
    
    /**
      * DataStreamDisplayArea - accepts data stream drops, adding them to its mdi area as it gets them
      *
      * @author Andy Pritchard
      */
    class DataStreamDisplayArea : public QMdiArea, public DataStreamDropListener {
        Q_OBJECT
        
    public:
        DataStreamDisplayArea(QWidget * parent = 0);
        void onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<int> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<float> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<Image> > stream);
        void dropEvent(QDropEvent * event);
        void dragEnterEvent(QDragEnterEvent * event);
        void addWindow(boost::shared_ptr<QWidget> content);
    };
    
    
    
    /**
      * Interface integration class.
      *
      * @author Andy Pritchard
      */
    class DataStreamPicker : public QDockWidget, public CauvBasicPlugin{
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)


    public:
        DataStreamPicker();
        virtual ~DataStreamPicker();

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;
        virtual void initialise(boost::shared_ptr<AUV>auv, boost::shared_ptr<CauvNode>node);
        
    private:
        Ui::DataStreamPicker *ui;
    };
    
}


/**
  * The QWidget subclass where the model data is displayed.
  *
  * It's outside the cauv namespace because its used as promoted widget in a ui file
  * TODO: find a way around this
  *
  * @author Andy Pritchard
  */
class DataStreamList : public QTreeWidget, public cauv::DataStreamDragSource {
    Q_OBJECT
public:
    DataStreamList(QWidget * parent);
    
    boost::shared_ptr<std::vector<boost::shared_ptr<cauv::DataStreamBase> > > getDataStreams() const;
    
private Q_SLOTS:
    void editStarted(QTreeWidgetItem* item, int column);
    void itemEdited(QTreeWidgetItem* item, int column);
};

#endif // DATASTREAMDISPLAYS_H
