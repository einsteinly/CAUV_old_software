#ifndef DATASTREAMDISPLAYS_H
#define DATASTREAMDISPLAYS_H

#include <QDockWidget>
#include <QVariant>
#include <QTreeView>
#include <QTreeWidgetItem>
#include <QMdiArea>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>
#include <model/auv_model.h>

#include "cauvinterfaceelement.h"
#include "datastreamdragging.h"

#include <cstdio>

namespace Ui {
    class DataStreamPicker;
}


namespace cauv {
    
    /**
      * A DataStreamTreeItemBase is used as the base class for one row in the
      * table of data streams.
      *
      * @author Andy Pritchard
      */
    class DataStreamTreeItemBase : public QObject, public QTreeWidgetItem {
        Q_OBJECT
    public:
        DataStreamTreeItemBase(boost::shared_ptr<DataStreamBase> stream, QTreeWidgetItem * parent);
        virtual ~DataStreamTreeItemBase(){}
        boost::shared_ptr<DataStreamBase> getDataStreamBase();
        virtual bool updateStream(QVariant& ) = 0;
        
    protected Q_SLOTS:
        void updateIcon(int cell, QImage image){
            this->setIcon(cell, QIcon(QPixmap::fromImage(image)));
        }

    Q_SIGNALS:
        void iconUpdated(int cell, QImage image);

    private:
        boost::shared_ptr<DataStreamBase> m_stream;
    };
    
    
    /**
      * A DataStreamTreeItem is responsible for updating the DataStream (if its
      * mutable) and updating the table when the DataStream's value changes.
      * It is also where the name cell is filled in.
      *
      * @author Andy Pritchard
      */
    template<class T>
    class DataStreamTreeItem : public DataStreamTreeItemBase {

    public:
        DataStreamTreeItem(boost::shared_ptr< DataStream<T> > stream, QTreeWidgetItem * parent) :
                DataStreamTreeItemBase(stream, parent), m_stream(stream) {
            stream->onUpdate.connect(boost::bind(&DataStreamTreeItem<T>::onChange, this, _1));
            this->setText(0, QString::fromStdString(stream->getName(false)));
            onChange(stream->latest());
        }
        
        T qVariantToValue(QVariant& v){
            std::string value = v.toString().toStdString();

            try {
                int unitsLength = m_stream->getUnits().length();
                // take out where the units should be
                std::string units = value.substr(value.length()-unitsLength, unitsLength);
                if(units == m_stream->getUnits()){
                    value = value.substr(0, value.length()-unitsLength);
                }
            } catch (std::out_of_range){ }

           return boost::lexical_cast<T>(value);
        }
        
        virtual bool updateStream(QVariant& value){
            if(!m_stream->isMutable()) return false;
            
            try {
                boost::shared_ptr<MutableDataStream<T> > stream = boost::shared_static_cast<MutableDataStream<T> >(m_stream);
                stream->set(qVariantToValue(value));
                info() << m_stream->getName() << " value changed to: " << value.toString().toStdString();
                return true;
            } catch (boost::bad_lexical_cast){
                std::cout << "bad value = " << value.toString().toStdString().c_str() << std::endl;
                printf("%s", value.toString().toStdString().c_str());
                info() << m_stream->getName() << " given a bad value:" << value.toString().toStdString();
                return false;
            }
        }
        
    protected:
        boost::shared_ptr< DataStream<T> > m_stream;
        
        void onChange(const T value) {
            std::stringstream stream;
            stream << value << m_stream->getUnits();
            this->setText(1, QString::fromUtf8(stream.str().c_str()));
        }
    };
    
    
    // partial specializations
    // need one for int8_t as it prints as a char not as an int, so we cast it
    // to int in the implementation before printing
    template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value);
    template<> void DataStreamTreeItem<Image>::onChange(const Image value);
    // another for int8_t for much the same reason but with lexical cast this time
    template<> int8_t DataStreamTreeItem<int8_t>::qVariantToValue(QVariant& value);
    // also need some for out types as lexical cast doesn't know what to do
    template<> floatYPR DataStreamTreeItem<floatYPR>::qVariantToValue(QVariant& value);
    template<> Image DataStreamTreeItem<Image>::qVariantToValue(QVariant& value);
    
    
    
    /**
      * DataStreamDisplayArea - accepts data stream drops, adding them to its mdi area as it gets them
      *
      * @author Andy Pritchard
      */
    class DataStreamDisplayArea : public QMdiArea, public DataStreamDropListener, public CauvInterfaceElement {
        Q_OBJECT
        
    public:
        DataStreamDisplayArea(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual void initialise();
        void onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<int> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<float> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream);
        void onStreamDropped(boost::shared_ptr<DataStream<Image> > stream);
        void dropEvent(QDropEvent * event);
        void dragEnterEvent(QDragEnterEvent * event);
        void addWindow(QWidget * content);
    };
    
    
    
    /**
      * Interface integration class.
      *
      * @author Andy Pritchard
      */
    class DataStreamPicker : public QDockWidget, public CauvInterfaceElement {
        Q_OBJECT
    public:
        DataStreamPicker(const QString &name, boost::shared_ptr<cauv::AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual ~DataStreamPicker();
        virtual void initialise();
        
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
