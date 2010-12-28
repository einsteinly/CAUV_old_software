#ifndef DATA_STREAM_H_INCLUDED
#define DATA_STREAM_H_INCLUDED

#include <sstream>

#include <boost/signal.hpp>
#include <boost/function.hpp>

class DataSource {

    public:
        DataSource(const std::string name, DataSource* parent = NULL): m_parent(parent), m_name(name) {
        }

        virtual ~DataSource(){}

        virtual const std::string getName() {
            std::stringstream stream;

            if (m_parent != NULL)
                stream << m_parent->getName() << " " << m_name;
            else stream << m_name;

            return stream.str();
        }

    protected:

        DataSource* m_parent;
        std::string m_name;
};


/** A data stream represents one type of data being sent from a data source
*
* @author Andy Pritchard
*/
template <class T>

class DataStream : public DataSource {

    public:
        boost::signal<void(const T)> onUpdate;

        T m_latest;

        DataStream(const std::string name, DataSource* parent = NULL):DataSource(name, parent) {};

        virtual void update(const T data) {
            this->m_latest = data;
            this->onUpdate(data);
        }

        template <class S>
            void update(boost::function<T(S)> &getter, S input){
                update(getter(input));
        }

        virtual T latest() {
            return this->m_latest;
        }
};


/** A data stream represents one type of data being sent from a data source that can be changed
*
* @author Andy Pritchard
*/
template <class T>

class MutableDataStream : public DataStream<T> {

    public:
        boost::signal<void(const T)> onSet;

        MutableDataStream(const std::string name, DataSource* parent = NULL):DataStream<T>(name, parent) {};

        virtual void set(const T data) {
            this->update(data);
            this->onSet(data);
        }
};


#endif // DATA_STREAMS_H_INCLUDED
