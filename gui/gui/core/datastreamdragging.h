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

#ifndef DATASTREAMDRAGGING_H
#define DATASTREAMDRAGGING_H

#include <vector>
#include <boost/shared_ptr.hpp>
#include <common/data_stream.h>
#include <common/image.h>
#include <generated/types/floatYPR.h>
#include <generated/types/MotorDemand.h>

class QDropEvent;
class QDragEnterEvent;

namespace cauv {

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
        virtual void onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream);
        virtual void onStreamDropped(boost::shared_ptr<DataStream<int> > stream);
        virtual void onStreamDropped(boost::shared_ptr<DataStream<float> > stream);
        virtual void onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream);
        virtual void onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream);
        virtual void onStreamDropped(boost::shared_ptr<DataStream<Image> > stream);
        virtual void onStreamDropped(boost::shared_ptr<DataStream<MotorDemand> > stream);
    };
    
}

#endif // DATASTREAMDRAGGING_H
