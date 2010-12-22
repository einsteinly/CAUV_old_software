#ifndef DATA_STREAM_TOOLS_H_INCLUDED
#define DATA_STREAM_TOOLS_H_INCLUDED

#include "data_stream.h"

/** A DataStreamRecorder can be used to log a certain amount of input from a DataStream. Useful
* for graphing the data
*
* @author Andy Pritchard
*/
template<class T>

class DataStreamRecorder {

public:
    DataStreamRecorder<T>(boost::shared_ptr<DataStream<T> > stream, const unsigned int maximum = 1000):
            m_maximum(maximum), m_stream(stream) {
        stream->onUpdate.connect(boost::bind(&DataStreamRecorder<T>::change, this, _1));
    };

    const std::vector<T>& getHistory() const {
        return m_history;
    }

    void clear() {
        m_history.clear();
    }

    void change(const T &data) {
        if (!m_history.empty() && m_history.size() > m_maximum)
            m_history.erase(m_history.begin());

        m_history.push_back(data);
    };

protected:
    std::vector<T> m_history;
    unsigned int m_maximum;
    boost::shared_ptr<DataStream<T> > m_stream;
};



/** A DataStreamPrinter is used to print the data coming from a DataStream
*
* @author Andy Pritchard
*/
template<class T>

class DataStreamPrinter {

public:
    DataStreamPrinter<T>(boost::shared_ptr<DataStream<T> > stream, const std::ostream &output):
            m_stream(stream), m_ostream(output) {
        stream->onUpdate.connect(boost::bind(&DataStreamPrinter<T>::change, this, _1));
    };

    DataStreamPrinter<T>(boost::shared_ptr<DataStream<T> > stream):
            m_stream(stream), m_ostream(std::cout) {
        stream->onUpdate.connect(boost::bind(&DataStreamPrinter<T>::change, this, _1));
    };

    void change(const T &data) {
        m_ostream << data;
    };

protected:
    boost::shared_ptr<DataStream<T> > m_stream;
    std::ostream& m_ostream;
};


#endif // DATA_STREAM_TOOLS_H_INCLUDED
