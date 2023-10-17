#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink.h"

#include "ns3/applications-module.h"
#include <ctime>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DynamicNetworkSimulation");

std::string GenerateIPAddress(Ptr<Node> node) {

    Vector position = node->GetObject<MobilityModel> ()->GetPosition();

    double x = position.x;
    double y = position.y;
    double z = x * y + node->GetId();

    // Apply modulus to ensure values are within the range [0, 255]
    z = std::fmod(z, 256.0);

    return std::to_string(static_cast<int>(x + node->GetId())) + "." + std::to_string(static_cast<int>(y + node->GetId())) + "." + std::to_string(static_cast<int>(z)) + ".0";
}

uint16_t GeneratePort(Ptr<Node> node) {

    Vector position = node->GetObject<MobilityModel> ()->GetPosition();

    double x = position.x;
    double y = position.y;
    double z = x * y + node->GetId();

    // Apply modulus to ensure values are within the range [0, 255]
    z = std::fmod(z, 5000.0);

    return z;
}

void CreateNetwork(Ptr<Node> node, NodeContainer& otherNodes, Task task) {

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
  wifiPhy.SetChannel(wifiChannel.Create());

  // Add a mac and disable rate control
  WifiMacHelper wifiMac;

  wifiPhy.Set("TxPowerStart", DoubleValue(7.5));
  wifiPhy.Set("TxPowerEnd", DoubleValue(7.5));

  NodeContainer centerNode;
  centerNode.Add(node);

  // Install Wi-Fi on nodes
  wifiMac.SetType("ns3::AdhocWifiMac");
  NetDeviceContainer centerDevices = wifi.Install(wifiPhy, wifiMac, centerNode);
  NetDeviceContainer otherDevices = wifi.Install(wifiPhy, wifiMac, otherNodes);

  // Set up the Internet stack on all nodes
  InternetStackHelper internet;
  internet.Install(centerNode);
  internet.Install(otherNodes);

  // Assign IP addresses to devices
  Ipv4AddressHelper ipv4;
  std::string ipAddress = GenerateIPAddress(node);
  std::string subnetMask = "255.255.255.0";
  std::cout << ipAddress << "\n";
  std::cout << subnetMask << "\n";

  ipv4.SetBase(ipAddress.c_str(), subnetMask.c_str());
  Ipv4InterfaceContainer centerInterface = ipv4.Assign(centerDevices);
  Ipv4InterfaceContainer otherInterfaces = ipv4.Assign(otherDevices);

  uint16_t serverPort = GeneratePort(node);

  // Create a P2P link between each otherNode and the centerNode
  for (uint32_t i = 0; i < otherNodes.GetN(); ++i) {
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("100Mbps"));
    p2p.SetDeviceAttribute("Mtu", UintegerValue(1400));
    p2p.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));
    p2p.EnablePcapAll("p2p");

    NetDeviceContainer link = p2p.Install(otherNodes.Get(i), centerNode.Get(0));

    Address serverAddress(InetSocketAddress(otherInterfaces.GetAddress(i, 0), serverPort));

    // Create a simple UDP application
    UdpServerHelper server(serverPort);
    ApplicationContainer serverApps = server.Install(otherNodes.Get(i));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    UdpClientHelper client(serverAddress, serverPort);
    client.SetAttribute("MaxPackets", UintegerValue(1));
    client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    client.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = client.Install(centerNode.Get(0));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(10.0));
  }
}


std::queue<Task> GenerateTaskQueue() {
  std::queue<Task> taskQueue;
  Ptr<UniformRandomVariable> r_threads = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> r_ram = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> r_time = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> r_tasks = CreateObject<UniformRandomVariable> ();
  int n_tasks = r_tasks->GetInteger (5, 10);
  for (int i = 0; i < n_tasks; i++) {
    uint32_t threads = r_threads->GetInteger (4, 64);
    uint32_t ram = r_ram->GetInteger (12, 64);
    uint32_t time = r_time->GetInteger (1, 10);
    taskQueue.push(Task(threads, ram, time));
  }
  return taskQueue;
}

void PublishTask(Ptr<Node> node, NodeContainer& L1_nodes) {
  std::queue<Task> tqueue = node->GetTasks();
  if (!tqueue.empty()) {
    Task task = tqueue.front();

    CreateNetwork(node, L1_nodes, task);

    NS_LOG_INFO("Published Task: " << "Node=" << node->GetId() <<  " Threads=" << task.threads << " RAM=" << task.ram << " Time=" << task.time << " Tasks=" << tqueue.size());
    
    tqueue.pop();
    node->SetTasks(tqueue);
  }

  // Schedule the next task processing event
  Simulator::Schedule(Seconds(10), &PublishTask, node, L1_nodes);
}

void LogNodePositions_L1 (NodeContainer& nodes)
{
  for (uint32_t i = 0; i < nodes.GetN (); ++i)
  {
    Ptr<MobilityModel> mobility = nodes.Get (i)->GetObject<MobilityModel> ();
    Vector position = mobility->GetPosition ();
    NS_LOG_INFO ("Node L1 " << i << " Position: " << position);
  }

  // Schedule the next logging event
  Simulator::Schedule (Seconds (1), &LogNodePositions_L1, nodes);
}

void LogNodePositions_L2 (NodeContainer& nodes)
{
  for (uint32_t i = 0; i < nodes.GetN (); ++i)
  {
    Ptr<MobilityModel> mobility = nodes.Get (i)->GetObject<MobilityModel> ();
    Vector position = mobility->GetPosition ();
    NS_LOG_INFO ("Node L2 " << i << " Position: " << position);
  }

  // Schedule the next logging event
  Simulator::Schedule (Seconds (1), &LogNodePositions_L2, nodes);
}

int main (int argc, char *argv[])
{
  unsigned seed = std::time(0);
  SeedManager::SetSeed(seed);
  CommandLine cmd;
  cmd.Parse (argc, argv);

  Time::SetResolution (Time::NS);
  LogComponentEnable ("DynamicNetworkSimulation", LOG_LEVEL_ALL);
  //LogComponentEnable("TrafficControlLayer", LOG_LEVEL_ALL);
  //LogComponentEnable("PacketSink", LOG_LEVEL_ALL);
  LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);

  std::queue<Task> taskQueue = GenerateTaskQueue();

  Ptr<UniformRandomVariable> r_threads = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> r_ram = CreateObject<UniformRandomVariable> ();

  NodeContainer L1_nodes;
  for (int i = 0; i < 4; i++) {
    uint32_t threads = r_threads->GetInteger (1, 16);
    uint32_t ram = r_ram->GetInteger (4, 16);
    L1_nodes.Create (1, threads, ram);
  }

  NodeContainer L2_nodes;
  for (int i = 0; i < 2; i++) {
    uint32_t threads = r_threads->GetInteger (1, 16);
    uint32_t ram = r_ram->GetInteger (4, 16);
    std::queue<Task> queue = GenerateTaskQueue();
    L2_nodes.Create (1, threads, ram, queue);
    Simulator::Schedule(Seconds(1), &PublishTask, L2_nodes.Get (i), L1_nodes);
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

  mobility.Install (L1_nodes);
  mobility.Install (L2_nodes);

  //Simulator::Schedule (Seconds (1), &LogNodePositions_L1, std::ref(L1_nodes)); 
  //Simulator::Schedule (Seconds (1), &LogNodePositions_L2, std::ref(L2_nodes));  
  Simulator::Stop (Seconds (500));
  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}
