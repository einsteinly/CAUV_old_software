/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef TREEITEMS_H
#define TREEITEMS_H

#include <QVariant>
#include <QTreeWidgetItem>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>
#include <model/auv_model.h>

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
        void updateIcon(int cell, QImage &image);
        void updateIcon(int cell, const Image &image);
        void updateValue(const QString value);

    Q_SIGNALS:
        void iconUpdated(int cell, const Image &image);
        void valueUpdated(const QString value);

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
    class DataStreamTreeItem : public DataStreamTreeItemBase, public boost::signals2::trackable {

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
                info() << m_stream->getName() << " given a bad value:" << value.toString().toStdString();
                return false;
            }
        }

    protected:
        boost::shared_ptr< DataStream<T> > m_stream;

        void onChange(const T value) {
            std::stringstream stream;
            stream << value << m_stream->getUnits();
            Q_EMIT valueUpdated(QString::fromStdString(stream.str()));
        }
    };


    // partial specializations
    // need one for int8_t as it prints as a char not as an int, so we cast it
    // to int in the implementation before printing
    template<> void DataStreamTreeItem<int8_t>::onChange(const int8_t value);
    template<> void DataStreamTreeItem<Image>::onChange(const Image value);
    template<> void DataStreamTreeItem<floatYPR>::onChange(const floatYPR value);
    template<> void DataStreamTreeItem<float>::onChange(const float value);
    // another for int8_t for much the same reason but with lexical cast this time
    template<> int8_t DataStreamTreeItem<int8_t>::qVariantToValue(QVariant& value);
    // also need some for out types as lexical cast doesn't know what to do
    template<> floatYPR DataStreamTreeItem<floatYPR>::qVariantToValue(QVariant& value);
    template<> MotorDemand DataStreamTreeItem<MotorDemand>::qVariantToValue(QVariant& value);
    template<> Image DataStreamTreeItem<Image>::qVariantToValue(QVariant& value);

} // namespace cauv

#endif // TREEITEMS_H
