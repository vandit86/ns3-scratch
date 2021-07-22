/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

//
// This is an illustration of how one could use virtualization techniques to
// allow running applications on virtual machines talking over simulated
// networks.
//
// The actual steps required to configure the virtual machines can be rather
// involved, so we don't go into that here.  Please have a look at one of
// our HOWTOs on the nsnam wiki for more details about how to get the 
// system confgured.  For an example, have a look at "HOWTO Use Linux 
// Containers to set up virtual networks" which uses this code as an 
// example.
//
// The configuration you are after is explained in great detail in the 
// HOWTO, but looks like the following:

// LeftTap <----wifi-----> AP <----csma-------> right_tap 

//
#include <iostream>
#include <fstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/csma-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("MPTapCsmaVirtualMachineExample");

void ThroughputMonitor (FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon)
{
  flowMon->CheckForLostPackets();
  std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
  Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
  {
    Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
    std::cout<<"Flow ID                 : " << stats->first <<" ; "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress<<std::endl;
//  std::cout<<"Tx Packets = " << stats->second.txPackets<<std::endl;
//  std::cout<<"Rx Packets = " << stats->second.rxPackets<<std::endl;
    std::cout<<"Duration                : "<<stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds()<<std::endl;
    std::cout<<"Last Received Packet    : "<< stats->second.timeLastRxPacket.GetSeconds()<<" Seconds"<<std::endl;
    std::cout<<"Throughput: " << stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024/1024  << " Mbps"<<std::endl;
    std::cout<<"---------------------------------------------------------------------------"<<std::endl;
  }

 // Rescheduling the next call to this function to print throughput
 Simulator::Schedule(Seconds(3),&ThroughputMonitor, fmhelper, flowMon);

}

// Parse context strings of the form "/NodeList/x/DeviceList/x/..." to extract the NodeId integer
uint32_t
ContextToNodeId (std::string context)
{
  std::string sub = context.substr (10);
  uint32_t pos = sub.find ("/Device");
  return atoi (sub.substr (0, pos).c_str ());
}

void
PhyRxErrorTrace (std::string context, Ptr<const Packet> p, double snr)
{
  std::cout << "PHY-RX-ERROR time=" << Simulator::Now () << " node=" << ContextToNodeId (context)
            << " size=" << p->GetSize () << " snr=" << snr << std::endl;
}

void
PhyRxDropTrace (std::string context, Ptr<const Packet> p, WifiPhyRxfailureReason reason)
{
  std::cout << "PHY-RX-DROP time=" << Simulator::Now () << " node=" << ContextToNodeId (context)
            << " size=" << p->GetSize () << " reason=" << reason << std::endl;
}

int 
main (int argc, char *argv[])
{
  std::cout << "Start simulation "<< std::endl;
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);

  //Config::SetDefault ("ns3::DropTailQueue<Packet>::MaxSize", StringValue ("100p"));
  
  
  // We are interacting with the outside, real, world.  This means we have to 
  // interact in real-time and therefore means we have to use the real-time
  // simulator and take the time to calculate checksums.
  //
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  //
  // Create two ghost nodes.  The first will represent the virtual machine host
  // on the left side of the network; and the second will represent the VM on 
  // the right side.
  //
  NodeContainer nodes;
  nodes.Create (2);

  NodeContainer middle;
  middle.Create(1);     // create one node in the middle 

  NodeContainer left_m (nodes.Get(0), middle.Get(0));
  NodeContainer right_m (nodes.Get(1), middle.Get(0));

  // ./waf --run "tap=csma-virtual-machine --ns3::CsmaChannel::DataRate=10000000"
  //
  
  // // create default  wifi  channel 
  // YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  // YansWifiPhyHelper wifiPhy;
  // wifiPhy.SetChannel (wifiChannel.Create ());
  // // Add a mac and Set it to adhoc mode
  // WifiMacHelper wifiMac;
  // wifiMac.SetType ("ns3::AdhocWifiMac");
  // // wifi helper 
  // WifiHelper wifi;
  // wifi.SetStandard (WIFI_STANDARD_80211n_2_4GHZ);
  // Config::SetDefault ("ns3::LogDistancePropagationLossModel::ReferenceLoss", DoubleValue (40.046));
  // wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
  //                                       "DataMode", StringValue("HtMcs7"),
  //                                       "ControlMode", StringValue("HtMcs7"));
  

  // std::string phyMode ("DsssRate1Mbps");
  
  // // disable fragmentation for frames below 2200 bytes
  // Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold",
  //                     StringValue ("2200"));
  // // turn off RTS/CTS for frames below 2200 bytes
  // Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold",
  //                     StringValue ("2200"));
  // // Fix non-unicast data rate to be the same as that of unicast
  // Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
  //                     StringValue (phyMode));

  // Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));
  // YansWifiPhyHelper wifiPhy;

  // /** wifi channel **/
  // YansWifiChannelHelper wifiChannel;
  // wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // //wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  // double rss = -80; 
  // wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));

  // // create wifi channel
  // Ptr<YansWifiChannel> wifiChannelPtr = wifiChannel.Create ();
  // wifiPhy.SetChannel (wifiChannelPtr);

  /** Wifi PHY **/
  /***************************************************************************/
  // create default  wifi  channel 
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);  
  wifiPhy.SetChannel (wifiChannel.Create ());

  /** MAC layer **/
   // Add a mac and Set it to adhoc mode
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  
  // wifi helper 
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                StringValue ("OfdmRate54Mbps"), 
                                "ControlMode", StringValue ("OfdmRate24Mbps"));

  
  // set possition and mobility model 
  Ptr<ListPositionAllocator> positionAllocWifi = CreateObject<ListPositionAllocator> ();
  positionAllocWifi->Add (Vector (0.0, 0.0, 0.0));
  positionAllocWifi->Add (Vector (5, 0, 0));
  positionAllocWifi->Add (Vector (15, 0, 0));
  MobilityHelper mobility;
  mobility.SetPositionAllocator (positionAllocWifi);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (left_m.Get (0));
  mobility.Install (left_m.Get (1));
  mobility.Install (nodes.Get(1)); 

  // dev conteiner of wifi devices starting from left
  NetDeviceContainer leftDev = wifi.Install (wifiPhy, wifiMac, left_m);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds(1)));
  NetDeviceContainer rightDev = csma.Install (right_m);
  
  //internet for middle 
  InternetStackHelper inet; 
  inet.Install (middle);

  // Assign adress to AP wifi inface
  Ipv4AddressHelper ipv4h; 
  ipv4h.SetBase ("15.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer middle1iface = ipv4h.Assign (leftDev.Get(1));
  ipv4h.SetBase ("14.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer middle2iface = ipv4h.Assign (rightDev.Get(1));

  //
  // Use the TapBridgeHelper to connect to the pre-configured tap devices for
  // the left side.  We go with "UseBridge" mode since the CSMA devices support
  // promiscuous mode and can therefore make it appear that the bridge is
  // extended into ns-3.  The install method essentially bridges the specified
  // tap to the specified CSMA device.
  //
  TapBridgeHelper tapBridge;
  tapBridge.SetAttribute ("Mode", StringValue ("UseLocal"));

  // tapBridge.SetAttribute ("DeviceName", StringValue ("tap-left"));
  // tapBridge.Install (nodes.Get (0), leftDev.Get (0));
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-left-1"));
  tapBridge.Install (nodes.Get (0), leftDev.Get (0));

  //
  // Connect the right side tap to the right side CSMA device on the right-side
  // ghost node.
  //
  // tapBridge.SetAttribute ("DeviceName", StringValue ("tap-right"));
  // tapBridge.Install (nodes.Get (1), devices.Get (1));

  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-right-1"));
  tapBridge.Install (nodes.Get (1), rightDev.Get (0));

  // Trace PHY Rx error events
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/State/RxError",
                   MakeCallback (&PhyRxErrorTrace));
  // Trace PHY Rx drop events
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxDrop",
                   MakeCallback (&PhyRxDropTrace));

  //csma.EnablePcapAll ("mp-csma-3", true);
  // wifi ENABLE PCAP
  wifiPhy.EnablePcapAll ("mp-wifi-lte", true);

  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
  ConfigStore outputConfig2;
  outputConfig2.ConfigureDefaults ();
  outputConfig2.ConfigureAttributes ();

  std::cout << "Stop simulation \n";
  Simulator::Stop (Seconds (100.0));
  Simulator::Run ();
  Simulator::Destroy ();
  
  //flowMonitor->SerializeToXmlFile("mp-monitor.xml", true, true);
}
