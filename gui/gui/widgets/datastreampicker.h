#ifndef DATASTREAMPICKER_H
#define DATASTREAMPICKER_H

#include <QDockWidget>
#include <QVariant>
#include <QTreeView>

#include "../utils/treeitem.h"
#include "../cauvinterfaceelement.h"
#include "../datastreamdragging.h"

namespace Ui {
    class DataStreamPicker;
}

//outside the cauv namespace because its used a promoted widget in a ui file
// TODO: find a way around this
class DataStreamList : public QTreeView, public cauv::DataStreamDragSource {
public:
    DataStreamList(QWidget * parent) : QTreeView(parent){
    }

    boost::shared_ptr<std::vector<boost::shared_ptr<cauv::DataStreamBase> > > getDataStreams() const;
};

namespace cauv {


    class DataStreamTreeItemBase {
    public:
        DataStreamTreeItemBase(boost::shared_ptr<DataStreamBase> stream):
        m_stream(stream){}
        virtual ~DataStreamTreeItemBase(){}
        boost::shared_ptr<DataStreamBase> getDataStreamBase(){
            return m_stream;
        }

    private:
        boost::shared_ptr<DataStreamBase> m_stream;
    };

    template<class T>
    class DataStreamTreeItem : public TreeItem, public DataStreamTreeItemBase {

    public:
        DataStreamTreeItem(boost::shared_ptr< DataStream<T> > stream, TreeItem * parent) :
                TreeItem(parent), DataStreamTreeItemBase(stream), m_stream(stream) {
            m_name = stream->getName();
            stream->onUpdate.connect(boost::bind(&DataStreamTreeItem<T>::onChange, this, _1));
        }

        DataStreamTreeItem(boost::shared_ptr< DataStream<T> > stream, std::string name, TreeItem * parent) :
                TreeItem(parent), DataStreamTreeItemBase(stream), m_stream(stream), m_name(name) {
            stream->onUpdate.connect(boost::bind(&DataStreamTreeItem<T>::onChange, this, _1));
        }

        int columnCount() const {
            return 2;
        }

        Qt::ItemFlags flags(int column) const {
            Qt::ItemFlags flags = TreeItem::flags(column) | Qt::ItemIsDragEnabled | Qt::ItemIsSelectable;

            if(column == 1 && m_stream->isMutable()){
                flags = flags | Qt::ItemIsEditable;
            }

            return flags;
        }

        QVariant data(int column) const{
            if(column == 0){
                return QVariant(QString::fromStdString(m_name));
            } else {
                return QVariant(QString::fromStdString(m_value));
            }
        }

        void setValue(std::string value){
            m_value = value;
        }

    protected:
        boost::shared_ptr< DataStream<T> > m_stream;
        std::string m_name;
        std::string m_value;

        void onChange(const T value) {
            std::stringstream stream;
            stream << value;
            setValue(stream.str());
        }
    };

    // partial specialization for int8_t as it prints as a char not as an int
    // so we cast in to int in the implementation before printing
    template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value);



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

#endif // DATASTREAMPICKER_H
