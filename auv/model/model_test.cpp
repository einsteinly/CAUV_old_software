#include <iostream>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <common/data_stream_tools.h>

#include "auv_model.h"
#include "auv_controller.h"

using namespace std;
using namespace cauv;

int main()
{
    boost::shared_ptr< AUV > auv = boost::make_shared<AUV>();


    AUVController controller(auv);

    cout << controller.enabled() << endl;  // []     = 1
    controller.pushState(false);
    cout << controller.enabled() << endl;  // [0]    = 0
    controller.pushState(false);
    cout << controller.enabled() << endl;  // [00]   = 0
    controller.pushState(true);
    cout << controller.enabled() << endl;  // [001]  = 1
    controller.popState();
    cout << controller.enabled() << endl;  // [00]  =  0
    controller.popState();
    cout << controller.enabled() << endl;  // [0]   =  0
    controller.popState();
    cout << controller.enabled() << endl;  // []    =  1


    DataStreamRecorder<int8_t> recorder(auv->motors.prop, 100);
    DataStreamPrinter<int8_t> printer(auv->motors.prop);
    DataStreamPrinter<bool> printer2(auv->autopilots.bearing->enabled);
    auv->motors.prop->set(10);

    auv->autopilots.bearing->enabled->set(false);

    //cout << "Count: " << auv.m_allStreams.size() << endl;
/*
    AUV::DataStream<int>* ds = dynamic_cast<AUV::DataStream<int>*>(auv.m_allStreams.back());

    if(typeid(*ds) == typeid(AUV::DataStream<int>))
        cout << "true" << endl;
    else cout << "false" << endl;*/

    //auv.autopilots.HEADING.kvalues.kP.set(0.5f);

}
