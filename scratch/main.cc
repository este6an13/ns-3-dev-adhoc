#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DynamicNetworkSimulation");

int main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  Time::SetResolution (Time::NS);
  LogComponentEnable ("DynamicNetworkSimulation", LOG_LEVEL_INFO);

  NodeContainer nodes;
  nodes.Create (2); // Creating 10 nodes

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5.0),
                                 "DeltaY", DoubleValue (5.0),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::LevyFlight2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));

  mobility.Install (nodes);

  InternetStackHelper internet;
  internet.Install (nodes);

  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

  NetDeviceContainer devices;
  devices = p2p.Install (nodes.Get (0), nodes.Get (1));

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");

  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  Simulator::Stop (Seconds (10));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
