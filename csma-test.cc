#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/csma-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/config-store.h"


using namespace ns3;

// print ping  
static void
PingRtt (std::string context, Time rtt)
{
  NS_LOG_UNCOND ("Received Response with RTT = " << rtt);
}

/**  
 * \brief Used to print information of Flow monitor every second
*/
void
ThroughputMonitor (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon)
{
  flowMon->CheckForLostPackets ();
  std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats ();
  Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier ());
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin ();
       stats != flowStats.end (); ++stats)
    {
      Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
      std::cout << "Flow ID                 : " << stats->first << " ; " << fiveTuple.sourceAddress
                << " -----> " << fiveTuple.destinationAddress << std::endl;
      std::cout << "Tx Packets = " << stats->second.txPackets << std::endl;
      std::cout << "Rx Packets = " << stats->second.rxPackets << std::endl;
      std::cout << "Duration                : "
                << stats->second.timeLastRxPacket.GetSeconds () -
                       stats->second.timeFirstTxPacket.GetSeconds ()
                << std::endl;
      std::cout << "Last Received Packet    : " << stats->second.timeLastRxPacket.GetSeconds ()
                << " Seconds" << std::endl;
      std::cout << "Throughput: "
                << stats->second.rxBytes * 8.0 /
                       (stats->second.timeLastRxPacket.GetSeconds () -
                        stats->second.timeFirstTxPacket.GetSeconds ()) /
                       1024 / 1024
                << " Mbps" << std::endl;
      std::cout << "---------------------------------------------------------------------------"
                << std::endl;
    }

  // Rescheduling the next call to this function to print throughput
  Simulator::Schedule (Seconds (1), &ThroughputMonitor, fmhelper, flowMon);
}


// ****************************************************************************************************************
// ****************************************************************************************************************
//                                      MAIN
// ****************************************************************************************************************
// ****************************************************************************************************************

int
main (int argc, char *argv[])
{
  double simTime = 60 ; 
  //
  NodeContainer nodes; // Create two ghost nodes.
  nodes.Create (2);

  CsmaHelper csma; // Csma Helper

  InternetStackHelper inet; // internet stack helper
  inet.Install (nodes);

  // configure CSMA AP <--> REMOTE
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (1000)));
  // csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds ()));

  // Install devices on nodes from path #1
  NetDeviceContainer dev = csma.Install (nodes);
  
  // Assign adress 
  Ipv4AddressHelper ipv4h;                      // Ipv4 Address Helper
  ipv4h.SetBase ("16.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_ap_wifi = ipv4h.Assign (dev);

  // ****************************************************************************************************************
  //                        Configure APPLICATIONS
  // ****************************************************************************************************************

  std::cout<< "Create  Appliation" << std::endl;
  Ptr<V4Ping> app = CreateObject<V4Ping> ();
  app->SetAttribute ("Remote", Ipv4AddressValue ("16.0.0.2"));
  app->SetAttribute ("Verbose", BooleanValue (true));
  nodes.Get(0)->AddApplication (app);
  app->SetStartTime (Seconds (1.0));
  app->SetStopTime (Seconds (simTime-1));
  
  // Give the application a name.  This makes life much easier when constructing
  // config paths.
  Names::Add ("app", app);

  // Hook a trace to print something when the response comes back.
  Config::Connect ("/Names/app/Rtt", MakeCallback (&PingRtt));

  csma.EnablePcapAll("csma-test", true);

  /**
   * On/off application 
    */
  // Create the OnOff application to send UDP datagrams of size
  uint32_t packetSize = 1000;
  uint16_t port = 9;   // Discard port (RFC 863)

  // Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("1000"));
  // Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue ("54Mb/s")); 

  //  OnOffHelper onoff ("ns3::TcpSocketFactory",
  //                    InetSocketAddress (Ipv4Address ("16.0.0.1"), port));

  OnOffHelper onoff ("ns3::TcpSocketFactory", 
                     Address (InetSocketAddress (Ipv4Address ("16.0.0.1"), port)));
  onoff.SetConstantRate (DataRate ("100Mb/s"), packetSize); 
  
  ApplicationContainer apps = onoff.Install (nodes.Get (1));
  apps.Start (Seconds (1.0));
  apps.Stop (Seconds (simTime-1));

  // Create a packet sink to receive these packets
  PacketSinkHelper sink ("ns3::TcpSocketFactory",
                         Address (InetSocketAddress (Ipv4Address::GetAny (), port)));
  apps = sink.Install (nodes.Get (0));
  apps.Start (Seconds (1.0));
  //apps.Stop (Seconds (10.0));

  // Install Flow monitor  
  Ptr<FlowMonitor> flowMonitor;
  FlowMonitorHelper flowHelper;
  flowMonitor = flowHelper.Install(nodes); // monitor on left node 

  // scheduling throughput to be printed every 3 seconds
  ThroughputMonitor (&flowHelper, flowMonitor);


      // config output
  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
  ConfigStore outputConfig2;
  //outputConfig2.ConfigureDefaults ();
  outputConfig2.ConfigureAttributes ();

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
}