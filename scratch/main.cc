#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"

#include "levy-flight-2d-mobility-model.h"

using namespace ns3;

int main(int argc, char *argv[])
{
    CommandLine cmd;
    cmd.Parse(argc, argv);

    NodeContainer nodes;
    nodes.Create(1);

    // Install Internet stack
    InternetStackHelper internet;
    internet.Install(nodes);

    // Install Mobility model
    Ptr<LevyFlight2dMobilityModel> mobility = CreateObject<LevyFlight2dMobilityModel>();
    mobility->SetBounds(Rectangle(0.0, 100.0, 0.0, 100.0));
    mobility->SetStepSize(10.0);
    mobility->SetAlpha(2.0);

    nodes.Get(0)->AggregateObject(mobility);

    // Install a simple point-to-point link
    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    pointToPoint.SetChannelAttribute("Delay", StringValue("2ms"));

    NetDeviceContainer devices;
    devices = pointToPoint.Install(nodes);

    // Assign IP addresses
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // Create a packet sink at the receiver
    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", InetSocketAddress(interfaces.GetAddress(1)));
    ApplicationContainer sinkApps = packetSinkHelper.Install(nodes.Get(0));
    sinkApps.Start(Seconds(0.0));
    sinkApps.Stop(Seconds(10.0));

    // Create a UdpClient application to send UDP packets from node 0 to node 1
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(interfaces.GetAddress(1), 9));
    onoff.SetConstantRate(DataRate("1Mbps"));
    onoff.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer apps = onoff.Install(nodes.Get(0));
    apps.Start(Seconds(1.0));
    apps.Stop(Seconds(10.0));

    // Configure tracing
    AsciiTraceHelper ascii;
    pointToPoint.EnableAsciiAll(ascii.CreateFileStream("levy-flight-trace.tr"));

    Simulator::Stop(Seconds(10.0));

    // Run the simulation
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
