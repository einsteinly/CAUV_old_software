#ifndef DATASTREAMDRAGGING_H
#define DATASTREAMDRAGGING_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <gui/core/streams/streams.h>

// for types
#include <generated/messages_fwd.h>

class QDropEvent;
class QDragEnterEvent;

namespace cauv {
    namespace gui {

        class DataStreamDragSource{
        public:
            virtual boost::shared_ptr<std::vector<boost::shared_ptr<DataStreamBase> > > getDataStreams() = 0;
        };


        class DataStreamDropListener {
        public:
            virtual void dragEnterEvent(QDragEnterEvent *event);
            virtual void dropEvent(QDropEvent *event);

        protected:
            virtual bool routeStream(boost::shared_ptr<DataStreamBase> s);
            virtual void onStreamDropped(boost::shared_ptr<IntStream> stream);
            virtual void onStreamDropped(boost::shared_ptr<FloatStream> stream);

            //virtual void onStreamDropped(boost::shared_ptr<DataStream<float> > stream);
            //virtual void onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream);
            //virtual void onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream);
            //virtual void onStreamDropped(boost::shared_ptr<DataStream<Image> > stream);
            //virtual void onStreamDropped(boost::shared_ptr<DataStream<MotorDemand> > stream);
        };

    } // namespace gui
} // namespace cauv

#endif // DATASTREAMDRAGGING_H
