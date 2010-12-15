#ifndef DATA_STREAM_H_INCLUDED
#define DATA_STREAM_H_INCLUDED

#include <sstream>

#include <boost/signal.hpp>

class DataSource {

    public:
        DataSource(const std::string name, DataSource* parent = NULL): m_parent(parent), m_name(name) {
        }

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
        boost::signal<void(const T)> onSet;
        boost::signal<void(const T)> onUpdate;

        T m_latest;

        DataStream(const std::string name, DataSource* parent = NULL):DataSource(name, parent) {};

        virtual void set(const T data) {
            this->update(data);
            this->onSet(data);
        }

        virtual void update(const T data) {
            this->m_latest = data;
            this->onUpdate(data);
        }

        virtual T get() {
            return this->m_latest;
        }

};


#endif // DATA_STREAMS_H_INCLUDED
