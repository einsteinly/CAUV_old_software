/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#include "utility/async.h"

template<unsigned N>
void foo(){
    debug() << N;
}

int main(){
    {
        AsyncService s;
        s.callAfterMicroseconds(foo<0>, 0);
        s.callAfterMicroseconds(foo<100>, 100);
        s.callAfterMicroseconds(foo<10000>, 10000);
        s.callAfterMicroseconds(foo<10>, 10);
        s.callAfterMicroseconds(foo<1234>, 1234);
        s.callAfterMicroseconds(foo<12345>, 12345);
        s.callAfterMicroseconds(foo<12346>, 12346);
        for(volatile int i = 0; i < 1e8; i++)
            i=i;
        s.callAfterMicroseconds(foo<12347>, 12347);
        s.callAfterMicroseconds(foo<12348>, 12348);
        s.waitForCompletion();
    }

    {
        AsyncService s(4);
        s.callAfterMicroseconds(foo<0>, 0);
        s.callAfterMicroseconds(foo<200>, 100);
        s.callAfterMicroseconds(foo<20000>, 10000);
        s.callAfterMicroseconds(foo<20>, 10);
        s.callAfterMicroseconds(foo<2234>, 1234);
        s.callAfterMicroseconds(foo<22345>, 12345);
        s.callAfterMicroseconds(foo<22346>, 12346);
        for(volatile int i = 0; i < 1e8; i++)
            i=i;
        s.callAfterMicroseconds(foo<22347>, 12347);
        s.callAfterMicroseconds(foo<22348>, 12348);
        for(int i = 0; i < 10; i++)
            s.callAfterMicroseconds(foo<100>, 2000*i);
        s.cancelAllCallbacks();
        for(int i = 0; i < 10; i++)
            s.callAfterMicroseconds(foo<222>, 2000*i);
        //for(int i = 0; i < 100; i++)
        //    s.callAfterMicroseconds(foo<1000000>, i*1e6);
        s.waitForCompletion();
    }
}

