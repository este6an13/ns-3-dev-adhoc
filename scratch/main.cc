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

void CreateNetwork(Ptr<Node> node, NodeContainer& otherNodes) {

    // Assign IP addresses
    Ipv4AddressHelper address;
    std::string ipAddress = GenerateIPAddress(node);
    std::string subnetMask = "255.255.255.0";
    NS_LOG_INFO(ipAddress);
    NS_LOG_INFO(subnetMask);
    address.SetBase (ipAddress.c_str(), subnetMask.c_str());

    uint16_t serverPort = GeneratePort(node);
    
    for (uint32_t i = 0; i < otherNodes.GetN(); ++i) {
      // Create nodes
      NodeContainer nodes;

      uint32_t threads_cnode = node->GetThreads();
      uint32_t ram_cnode = node->GetRAM();
      std::queue<Task> tasks_cnode = node->GetTasks();

      // Extracting threads, ram, and tasks from otherNode
      uint32_t threads_onode = otherNodes.Get (i)->GetThreads();
      uint32_t ram_onode = otherNodes.Get (i)->GetRAM();
      std::queue<Task> tasks_onode = otherNodes.Get (i)->GetTasks();

      // Create vectors to pass to NodeContainer::Create
      std::vector<uint32_t> threads = {threads_cnode, threads_onode};
      std::vector<uint32_t> rams = {ram_cnode, ram_onode};
      std::vector<std::queue<Task>> tasks = {tasks_cnode, tasks_onode};

      // Call NodeContainer::Create
      nodes.Create(2, threads, rams, tasks);
      
      // Create p2p link
      PointToPointHelper p2p;
      p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
      p2p.SetChannelAttribute ("Delay", StringValue ("2ms"));

      NetDeviceContainer devices;
      devices = p2p.Install (nodes);

      // Install internet stack
      InternetStackHelper stack;
      stack.Install (nodes);

      Ipv4InterfaceContainer interfaces = address.Assign (devices);
      NS_LOG_INFO(interfaces.GetAddress (0) << " - " << interfaces.GetAddress (1));

      // Enable pcap tracing
      p2p.EnablePcap ("pcap/p2p-" + std::to_string(node->GetId()) + "-" + std::to_string(i), devices, true);

      // Create a simple UDP application
      UdpServerHelper server (serverPort);
      ApplicationContainer serverApps = server.Install (nodes.Get (1));
      serverApps.Start (Seconds (1.0));
      serverApps.Stop (Seconds (5.0));

      UdpClientHelper client (interfaces.GetAddress (1), serverPort);
      client.SetAttribute ("MaxPackets", UintegerValue (1));
      client.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
      client.SetAttribute ("PacketSize", UintegerValue (1024));

      ApplicationContainer clientApps = client.Install (nodes.Get (0));
      clientApps.Start (Seconds (2.0));
      clientApps.Stop (Seconds (5.0));
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

    CreateNetwork(node, L1_nodes);

    NS_LOG_INFO("Published Task: " << "Node=" << node->GetId() <<  " Threads=" << task.threads << " RAM=" << task.ram << " Time=" << task.time << " Tasks=" << tqueue.size());
    
    tqueue.pop();
    node->SetTasks(tqueue);
  }

  // Schedule the next task processing event
  Simulator::Schedule(Seconds(20), &PublishTask, node, L1_nodes);
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

  std::queue<Task> taskQueue = GenerateTaskQueue();

  Ptr<UniformRandomVariable> r_threads = CreateObject<UniformRandomVariable> ();
  Ptr<UniformRandomVariable> r_ram = CreateObject<UniformRandomVariable> ();

  int N1 = 4;
  int N2 = 2;

  NodeContainer L1_nodes;
  for (int i = 0; i < N1; i++) {
    uint32_t threads = r_threads->GetInteger (1, 16);
    uint32_t ram = r_ram->GetInteger (4, 16);
    L1_nodes.Create (1, threads, ram);
  }

  NodeContainer L2_nodes;
  for (int i = 0; i < N2; i++) {
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
