/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/internet-module.h>
#include <ns3/point-to-point-module.h>
#include <ns3/applications-module.h>
#include <ns3/csma-helper.h>
#include <ns3/tap-bridge-helper.h>
#include <ns3/nstime.h>
#include <ns3/trace-helper.h>

NS_LOG_COMPONENT_DEFINE ("zmq_test");

int
main (int argc, char *argv[])
{
    //ns3::Packet::EnablePrinting();
    ns3::GlobalValue::Bind("SimulatorImplementationType",
                           ns3::StringValue("ns3::RealtimeSimulatorImpl"));
    ns3::NodeContainer nodes;
    nodes.Create (2);

    ns3::NetDeviceContainer devices;

    ns3::CsmaHelper csma;

    csma.SetChannelAttribute ("Delay", ns3::StringValue("100ms"));
    //csma.SetChannelAttribute ("DataRate", ns3::StringValue("1000kbps"));
    devices = csma.Install(nodes);

    //PointToPointHelper pointToPoint;
    //pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
    //pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

    ns3::InternetStackHelper inet_stack;
    inet_stack.Install(nodes);

    ns3::Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");

    ns3::Ipv4InterfaceContainer interfaces = address.Assign (devices);

    ns3::TapBridgeHelper tap1(interfaces.GetAddress(0));
    tap1.SetAttribute("Mode", ns3::StringValue("ConfigureLocal"));
    tap1.SetAttribute("DeviceName", ns3::StringValue("tap0"));
    tap1.Install(nodes.Get(0), devices.Get(0));

    ns3::TapBridgeHelper tap2(interfaces.GetAddress(1));
    tap2.SetAttribute("Mode", ns3::StringValue("ConfigureLocal"));
    tap2.SetAttribute("DeviceName", ns3::StringValue("tap1"));
    tap2.Install(nodes.Get(1), devices.Get(1));

    //ns3::AsciiTraceHelper ascii;
    //csma.EnableAsciiAll(ascii.CreateFileStream("realtime-ns-test.tr"));
    csma.EnablePcapAll("test.pcap", false);

    ns3::Simulator::Stop(ns3::Seconds(20));
    ns3::Simulator::Run();
    ns3::Simulator::Destroy();
    return 0;
}
