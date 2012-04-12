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

#include "client.h"

#include <debug/cauv_debug.h>
#include <utility/time.h>

#include <generated/types/TimeStamp.h>

#include <opencv2/highgui/highgui.hpp>

using namespace cauv;

int main(int, char**){
    try{
        CameraServerConnection C;
        
        {
            {boost::shared_ptr<ImageWrapper> a = C.getImage(0, 640, 480);}
            {boost::shared_ptr<ImageWrapper> b = C.getImage(0, 640, 480);}
            boost::shared_ptr<ImageWrapper> c = C.getImage(0, 640, 480);
            boost::shared_ptr<ImageWrapper> d = C.getImage(0, 640, 480);
            boost::shared_ptr<ImageWrapper> e = C.getImage(0, 640, 480);
            {boost::shared_ptr<ImageWrapper> f = C.getImage(0, 640, 480);}
            boost::shared_ptr<ImageWrapper> g = C.getImage(0, 640, 480);
            boost::shared_ptr<ImageWrapper> h = C.getImage(0, 640, 480);
        }
        
        {
            {boost::shared_ptr<ImageWrapper> a = C.getImage(0, 640, 480);}
            {boost::shared_ptr<ImageWrapper> b = C.getImage(0, 320, 240);}
            {boost::shared_ptr<ImageWrapper> c = C.getImage(0, 100, 200);}
            {boost::shared_ptr<ImageWrapper> d = C.getImage(0, 100, 200);}
            boost::shared_ptr<ImageWrapper> e = C.getImage(0, 320, 240);
            boost::shared_ptr<ImageWrapper> f = C.getImage(0, 320, 240);
            boost::shared_ptr<ImageWrapper> g = C.getImage(0, 320, 240);
        }

        uint32_t N = 100;
        boost::shared_ptr<ImageWrapper> t;
        cv::namedWindow("client test", CV_WINDOW_KEEPRATIO);
        TimeStamp tstart = now();
        for(uint32_t i = 0; i < N; i++){
            t = C.getImage(0, 320, 240);
            cv::imshow("client test", t->mat());
        }
        TimeStamp tend = now();
        uint64_t musecdiff = tend.musecs - tstart.musecs + 1000000*(tend.secs - tstart.secs);
        debug() << (1e6 * N) / musecdiff << "fps";

    }catch(std::exception& e){
        error() << e.what();
    }
    return 0;
}
