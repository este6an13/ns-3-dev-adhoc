#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DynamicNetworkSimulation");

void LogNodePositions (NodeContainer& nodes)
{
  for (uint32_t i = 0; i < nodes.GetN (); ++i)
  {
    Ptr<MobilityModel> mobility = nodes.Get (i)->GetObject<MobilityModel> ();
    Vector position = mobility->GetPosition ();
    NS_LOG_INFO ("Node " << i << " Position: " << position);
  }

  // Schedule the next logging event
  Simulator::Schedule (Seconds (1), &LogNodePositions, nodes);
}

int main (int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse (argc, argv);

  Time::SetResolution (Time::NS);
  LogComponentEnable ("DynamicNetworkSimulation", LOG_LEVEL_INFO);

  NodeContainer nodes;
  Ptr<UniformRandomVariable> r_threads = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> r_ram = CreateObject<UniformRandomVariable> ();
  for (int i = 0; i < 100; i++) {
    uint32_t threads = r_threads->GetInteger (1, 16);
    uint32_t ram = r_ram->GetInteger (4, 16);
    nodes.Create (1, threads, ram);
  }

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                  "X",
                                  StringValue("100.0"),
                                  "Y",
                                  StringValue("100.0"),
                                  "Rho",
                                  StringValue("ns3::UniformRandomVariable[Min=0|Max=30]"));

  mobility.SetMobilityModel ("ns3::LevyFlight2dMobilityModel");

  mobility.Install (nodes);

  Simulator::Schedule (Seconds (1), &LogNodePositions, std::ref(nodes));  
  Simulator::Stop (Seconds (100));
  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}
