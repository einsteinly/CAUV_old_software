#ifndef DATA_STREAM_TOOLS_H_INCLUDED
#define DATA_STREAM_TOOLS_H_INCLUDED

#include "data_stream.h"

 // don't like having htis here, but its needed for the types flaotXYZ and flaotYPR
// the DataStreamSplitter specialisations could be moved somewhere more sensible...
#include <generated/messages.h>


#include <iostream>

#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

namespace cauv {

    /** A DataStreamSplitter can be used to separate streams of structured data
    *   must be extended to be used
    *
    * @author Andy Pritchard
    */
    template<class T>

    class DataStreamSplitter {};

    /* floatYPR specialization */
    template<>

    class DataStreamSplitter<cauv::floatYPR> {

    public:
        DataStreamSplitter<cauv::floatYPR>(boost::shared_ptr<DataStream<cauv::floatYPR> > stream) :
                yaw(boost::make_shared<DataStream<float> >("Yaw")),
                pitch(boost::make_shared<DataStream<float> >("Pitch")),
                roll(boost::make_shared<DataStream<float> >("Roll"))
        {
            // getter function binds
            boost::function<float(cauv::floatYPR)> yawGetter = boost::bind(&cauv::floatYPR::yaw, _1);
            boost::function<float(cauv::floatYPR)> pitchGetter = boost::bind(&cauv::floatYPR::pitch, _1);
            boost::function<float(cauv::floatYPR)> rollGetter = boost::bind(&cauv::floatYPR::roll, _1);

            // connect up the slots
            // the getter functions are passed to the new data streams and evaluated there
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatYPR>, yaw.get(), yawGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatYPR>, pitch.get(), pitchGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatYPR>, roll.get(), rollGetter, _1));
        }

        boost::shared_ptr<DataStream<float> > yaw;
        boost::shared_ptr<DataStream<float> > pitch;
        boost::shared_ptr<DataStream<float> > roll;
    };

    /* floatXYZ specialization */
    template<>

    class DataStreamSplitter<cauv::floatXYZ> {

    public:
        DataStreamSplitter<cauv::floatXYZ>(boost::shared_ptr<DataStream<cauv::floatXYZ> > stream) :
                x(boost::make_shared<DataStream<float> >("X")),
                y(boost::make_shared<DataStream<float> >("Y")),
                z(boost::make_shared<DataStream<float> >("Z"))
        {
            // getter function binds
            boost::function<float(cauv::floatXYZ)> xGetter = boost::bind(&cauv::floatXYZ::x, _1);
            boost::function<float(cauv::floatXYZ)> yGetter = boost::bind(&cauv::floatXYZ::y, _1);
            boost::function<float(cauv::floatXYZ)> zGetter = boost::bind(&cauv::floatXYZ::z, _1);

            // connect up the slots
            // the getter functions are passed to the new data streams and evaluated there
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatXYZ>, x.get(), xGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatXYZ>, y.get(), yGetter, _1));
            stream->onUpdate.connect(boost::bind(&DataStream<float>::update<cauv::floatXYZ>, z.get(), zGetter, _1));
        }

        boost::shared_ptr<DataStream<float> > x;
        boost::shared_ptr<DataStream<float> > y;
        boost::shared_ptr<DataStream<float> > z;
    };


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

}

#endif // DATA_STREAM_TOOLS_H_INCLUDED
