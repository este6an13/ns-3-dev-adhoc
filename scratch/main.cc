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

// Serialization function
void SerializeTask(const Task& task, Buffer& buffer) {
    buffer.AddAtEnd(3 * sizeof(uint32_t));
    Buffer::Iterator iter = buffer.Begin();
    iter.WriteU32(task.threads);
    iter.WriteU32(task.ram);
    iter.WriteU32(task.time);
}

// Deserialization function
Task DeserializeTask(Buffer::Iterator& iter) {
  uint32_t threads = iter.ReadU32();
  uint32_t ram = iter.ReadU32();
  uint32_t time = iter.ReadU32();
  return Task(threads, ram, time);
}

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

void HandleRead(Ptr<Socket> socket) {
    Ptr<Packet> packet = socket->Recv();
    // Process the received data here
    NS_LOG_DEBUG("Received data: " << packet->GetSize() << " bytes");
}

// Helper function to send data after a delay
void SendData(Ptr<Node> node, Task task, Address serverAddress) {

  // Serialize the task and send it using UDP
  Buffer buffer;
  SerializeTask(task, buffer);

  uint8_t data[buffer.GetSize()];
  buffer.CopyData(data, buffer.GetSize());

  Ptr<Socket> socket = Socket::CreateSocket(node, UdpSocketFactory::GetTypeId());

  socket->SetRecvCallback(MakeCallback(&HandleRead));

  socket->Connect(serverAddress);

  // Send the data using Socket::Send
  uint32_t bytesSent = socket->Send(data, buffer.GetSize(), 0);

  // Check if the send operation was successful
  if (bytesSent == buffer.GetSize()) {
    NS_LOG_DEBUG("Data sent successfully - Bytes sent: " << bytesSent);
  } else {
    NS_LOG_WARN("Failed to send all data - Bytes sent: " << bytesSent);
  }
}

void CreateWiFiNetwork(Ptr<Node> node, NodeContainer& otherNodes, Task task) {
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

  for (uint32_t i = 0; i < otherNodes.GetN(); ++i) {
    Address serverAddress(InetSocketAddress(otherInterfaces.GetAddress(i, 0), serverPort));

    // Create a UDP server to receive data
    PacketSinkHelper packetSinkHelper("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), serverPort));
    ApplicationContainer serverApps = packetSinkHelper.Install(otherNodes.Get(i));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0)); // Adjust the stop time as needed

    // Create a UDP client on the centerNode to send data
    Address clientAddress(InetSocketAddress(centerInterface.GetAddress(0), serverPort));
    OnOffHelper onoff("ns3::UdpSocketFactory", clientAddress);
    onoff.SetAttribute("PacketSize", UintegerValue(1024)); // Adjust packet size as needed
    //onoff.SetAttribute("Remote", AddressValue(serverAddress)); // Set the server address
    ApplicationContainer clientApps = onoff.Install(node);
    //clientApps.Start(Seconds(1.0));
    //clientApps.Stop(Seconds(5.0));

    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    wifiPhy.EnablePcap("pcap/center-" + std::to_string(node->GetId()), centerDevices.Get(0), true);
    wifiPhy.EnablePcap("pcap/others-" + std::to_string(i), otherDevices.Get(i), true);

    // Wait for some time before sending the data
    //Simulator::Schedule(Seconds(20.0), &SendData, node, task, serverAddress);
    //Simulator::Schedule(Seconds(20.0), &SendData, node, task, serverAddress);
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

    CreateWiFiNetwork(node, L1_nodes, task);

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
