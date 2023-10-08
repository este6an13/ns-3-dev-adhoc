#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("DynamicNetworkSimulation");

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

void PublishTask(Ptr<Node> node) {
  std::queue<Task> tqueue = node->GetTasks();
  if (!tqueue.empty()) {
    Task task = tqueue.front();
    NS_LOG_INFO("Published Task: " << "Node=" << node->GetId() <<  " Threads=" << task.threads << " RAM=" << task.ram << " Time=" << task.time << " Tasks=" << tqueue.size());
    tqueue.pop();
    node->SetTasks(tqueue);
  }

  // Schedule the next task processing event
  Simulator::Schedule(Seconds(1), &PublishTask, node);
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
  CommandLine cmd;
  cmd.Parse (argc, argv);

  Time::SetResolution (Time::NS);
  LogComponentEnable ("DynamicNetworkSimulation", LOG_LEVEL_INFO);

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
    Simulator::Schedule(Seconds(1), &PublishTask, L2_nodes.Get (i));
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

  Simulator::Schedule (Seconds (1), &LogNodePositions_L1, std::ref(L1_nodes)); 
  Simulator::Schedule (Seconds (1), &LogNodePositions_L2, std::ref(L2_nodes));  
  //Simulator::Schedule(Seconds(1), &PublishTask, std::ref(taskQueue));
  Simulator::Stop (Seconds (100));
  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}
