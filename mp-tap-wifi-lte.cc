/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
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
 *
 * Author: Jaume Nin <jaume.nin@cttc.cat>
 */

#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
//#include "ns3/gtk-config-store.h"
// Tapbridge to allow connecting to the real world
#include "ns3/tap-bridge-module.h"
#include "ns3/csma-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/internet-apps-module.h"


// wifi 
#include "ns3/wifi-module.h"


using namespace ns3;

/**
 * Sample simulation script for LTE+EPC. It instantiates several eNodeB,
 * attaches one UE per eNodeB starts a flow for each UE to  and from a remote host.
 * It also  starts yet another flow between each UE pair.
 */

NS_LOG_COMPONENT_DEFINE ("EpcFirstExample");

void ThroughputMonitor (FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon)
{
  flowMon->CheckForLostPackets();
  std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
  Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
  {
    Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
    std::cout<<"Flow ID                 : " << stats->first <<" ; "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress<<std::endl;
    std::cout << "Tx Packets = " << stats->second.txPackets << std::endl;
    std::cout << "Rx Packets = " << stats->second.rxPackets << std::endl;
    std::cout<<"Duration                : "<<stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds()<<std::endl;
    std::cout<<"Last Received Packet    : "<< stats->second.timeLastRxPacket.GetSeconds()<<" Seconds"<<std::endl;
    std::cout<<"Throughput: " << stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024/1024  << " Mbps"<<std::endl;
    std::cout<<"---------------------------------------------------------------------------"<<std::endl;
  }

 // Rescheduling the next call to this function to print throughput
 Simulator::Schedule(Seconds(3),&ThroughputMonitor, fmhelper, flowMon);
  
}

/**
   * Set the address of a previously added UE
   * \brief  we need to add address of lxc container connected through wired (CSMA) connection to the UE because in LENA, in 
   * the downlink, the PGW uses the destination IP address to identify a single UE and the eNB to which it is attached to. 
   * If you specify a destination IP address that does not belong to any UE, 
   * the packet will just be dropped by the PGW, because it would not know to which eNB it should be routed 
   * through
   * \param pgw PGW node 
   * \param ueLteNetDev lteNetDevice installed on the UE (nedded to get IMSI)
   * \param ueAddr the IPv4 address of the LXC container connected to ue Net device
*/
void
addBackAddress (Ptr<Node> pgw, Ptr<NetDevice> ueLteNetDev, Ipv4Address addr)
{
  // First we need to get IMSI of ueLteNetDevice conencted through csma link to container
  Ptr<LteUeNetDevice> uedev = ueLteNetDev->GetObject<LteUeNetDevice> ();
  uint64_t imsi = uedev->GetImsi ();

  // get PGW application from pgw node
  Ptr<EpcPgwApplication> pgwApp = pgw->GetApplication (0)->GetObject<EpcPgwApplication> ();

  // add container address to allow traffic through PGW node
  pgwApp->SetUeAddress (imsi, addr);
}


/* 
  Trace CSMA packet drop 
 */
void
CsmaPhyTxDrop (std::string context, Ptr<const Packet> p)
{
  //Ptr <NetDevice> ndev = GetNetDeviceFromContext (context);

  std::cout << p->GetUid() << std::endl << " at " << Simulator::Now(); 
}

// ****************************************************************************************************************
// ****************************************************************************************************************
//                                      MAIN
// ****************************************************************************************************************
// ****************************************************************************************************************

int
main (int argc, char *argv[])
{
  // uint16_t numberOfNodes = 1;
  double simTime = 60;    // sec
  double distance = 60.0; // m
  bool useCa = false;
  uint64_t path2delay = 1; // delay between AP and remote host 

  // Command line arguments
  CommandLine cmd;
  cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue("distance", "Distance between eNBs [m]", distance);
  cmd.AddValue("useCa", "Whether to use carrier aggregation.", useCa);
  cmd.AddValue("path2delay", "delay between AP and remote host on second path [us]", path2delay);
  cmd.Parse(argc, argv);

   // ****************************************
  // Global configurations  
  // ****************************************
  //
  // We are interacting with the outside, real, world.  This means we have to
  // interact in real-time and therefore means we have to use the real-time
  // simulator and take the time to calculate checksums.
  //
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
  // Config::SetDefault ("ns3::RealtimeSimulatorImpl::SynchronizationMode"=StringValue("HardLimit"));

  // config output
  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));


  // Create ghost nodes that will host the TapBridge and connect to the base system
  NodeContainer nodes;
  nodes.Create (2);

  NodeContainer nodeAP;         // create one wifi AP node
  nodeAP.Create(1);                   
  
  NodeContainer ueNode;         // create one UE node
  ueNode.Create(1);                   

  NodeContainer enbNode;       // create one eNB node     
  enbNode.Create (1);

  // ****************************************
  // Helpers used in simulation 
  // ****************************************
  MobilityHelper mobility;                      // mobility helper 
  InternetStackHelper inet;                     // internet stack helper 
  YansWifiChannelHelper wifiChannel;            // Yans Wifi Channel Helper
  YansWifiPhyHelper wifiPhy;                    // Yans Wifi Phy Helper
  WifiMacHelper wifiMac;                        // Wifi Mac Helper
  WifiHelper wifi;                              // Wifi Helper
  CsmaHelper csma;                              // Csma Helper
  Ipv4AddressHelper ipv4h;                      // Ipv4 Address Helper
  Ipv4StaticRoutingHelper ipv4RoutingHelper;    // Ipv4 Static Routing Helper

  // ****************************************
  // Add a default internet stack to the node (ARP, IPv4, ICMP, UDP and TCP).
  // ****************************************
  inet.Install (nodeAP);
  inet.Install (ueNode);

  // ****************************************************************************************************************
  //                                            MOBILITY MODEL
  // ****************************************************************************************************************
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0, 0, 0)); // pos of left
  positionAlloc->Add (Vector (5, 0, 5)); // pos of AP
  positionAlloc->Add (Vector (25, 0, 0)); // pos of right
  positionAlloc->Add (Vector (distance, 0, 10)); // pos of gNodeB

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes.Get (0));
  mobility.Install (nodeAP.Get (0));
  mobility.Install (nodes.Get (1));
  mobility.Install (enbNode.Get (0));


// ****************************************************************************************************************
  //                                  Configure PATH 1: WI-FI and CSMA 
  // ****************************************************************************************************************

  // create default  wifi  channel 
  wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);  // allow pcap traces 
  wifiPhy.SetChannel (wifiChannel.Create ());                         // create channel 

   // Add a mac and Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  
  // wifi helper set params 
  wifi.SetStandard (WIFI_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                StringValue ("OfdmRate54Mbps"), 
                                "ControlMode", StringValue ("OfdmRate24Mbps"));

 
  // dev conteiner of wifi devices starting from left 
  NodeContainer wifiNodes (nodes.Get(0), nodeAP.Get(0)); 
  NetDeviceContainer wifiDevices =  wifi.Install (wifiPhy, wifiMac, wifiNodes);

  // Assign adress to AP wifi inface
  ipv4h.SetBase ("15.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer wifiAPinface = ipv4h.Assign (wifiDevices.Get(1)); // #1 inface AP

  // Copnnect AP with right host through csma
  csma.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("100Mb/s")));
  csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (path2delay)));
  NodeContainer csma_AP_right_nodes (nodeAP.Get (0), nodes.Get (1));
  NetDeviceContainer csma_AP_right_devices = csma.Install (csma_AP_right_nodes);

  // Assign adress to AP csma inface
  ipv4h.SetBase ("14.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer csmaAPinface = ipv4h.Assign (csma_AP_right_devices.Get (0)); // #2 inface AP

  // ****************************************************************************************************************
  //                                  Configure PATH 2: LTE and CSMA
  // ****************************************************************************************************************

  // LTE defoult config ...
  {
    // Default LTE configuration
    Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::RrFfMacScheduler::HarqEnabled", BooleanValue (false));
    Config::SetDefault ("ns3::LteHelper::PathlossModel",
                        StringValue ("ns3::FriisSpectrumPropagationLossModel"));
    Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue (100)); //20MHz bandwidth
    Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue (100)); //20MHz bandwidth
    Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));

    SeedManager::SetSeed ((uint32_t) (time (NULL)));

    // Uncomment to enable PCAP tracing
    // Config::SetDefault ("ns3::PointToPointEpcHelper::S1uLinkEnablePcap", BooleanValue (true));

    if (useCa)
      {
        Config::SetDefault ("ns3::LteHelper::UseCa", BooleanValue (useCa));
        Config::SetDefault ("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue (2));
        Config::SetDefault ("ns3::LteHelper::EnbComponentCarrierManager",
                            StringValue ("ns3::RrComponentCarrierManager"));
        Config::SetDefault ("ns3::ComponentCarrier::UlBandwidth", UintegerValue (100));
        Config::SetDefault ("ns3::ComponentCarrier::DlBandwidth", UintegerValue (100));
        Config::SetDefault ("ns3::ComponentCarrier::PrimaryCarrier", BooleanValue (true));
      }
  }
  
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // Get the pgw node to install the csma device
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Install LTE Devices to the nodes
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNode.Get(0));
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNode.Get(0));  // 

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));  // #1 UE iface

  // Attach one UE per eNodeB
  // side effect: the default EPS bearer will be activated
  lteHelper->Attach (ueLteDevs.Get (0), enbLteDevs.Get (0));

  // Setup the left ghost node and hook it to the first UE
  csma.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s"))); // Set high to avoid impact of this link
  csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (1)));
  NodeContainer csma_left_nodes(nodes.Get(0), ueNode.Get(0));
  NetDeviceContainer csma_left_devices = csma.Install (csma_left_nodes);

  // Setup the right ghost node and hook it to the remote node
  csma.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("100Mb/s"))); // Set high to avoid impact of this link
  csma.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (1)));
  NodeContainer csma_right_nodes (nodes.Get (1), pgw);
  NetDeviceContainer csma_right_devices = csma.Install (csma_right_nodes);

  // Configuring the UE and the csma device in the pgw as the default gateways for the external networks
  ipv4h.SetBase ("11.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer UeIpIfaces = ipv4h.Assign (NetDeviceContainer (csma_left_devices.Get (1)));  // #2

  ipv4h.SetBase ("13.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer rNodeIpIfaces =
      ipv4h.Assign (NetDeviceContainer (csma_right_devices.Get (1)));

    // ADD lxc mp-left container address to allow "ping" through EPC-PGW node
  addBackAddress (pgw, ueLteDevs.Get (0), Ipv4Address ("11.0.0.2"));

  
  
  // Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/$ns3::CsmaNetDevice/PhyTxDrop",
  //                          MakeCallback (&CsmaPhyTxDrop));
  
  // Config::ConnectFailSafe ("/NodeList/*/DeviceList/*/$ns3::CsmaNetDevice/PhyRxDrop",
  //                          MakeCallback (&CsmaPhyTxDrop));


  // add route to left lxc
  // ip route add 14.0.0.0/8 via 15.0.0.1 dev eth1
  // add route to right lxc
  // ip route add 15.0.0.0/8 via 14.0.0.1 dev eth1

  // ********************************************************
  // Configuring TAP bridge
  // ********************************************************
  
  // Now we set up the tap bridges in the ghost nodes to connect to an external tap defined in the OS (and hooked to a namespace)
  TapBridgeHelper tapBridge;
  // left containet
  tapBridge.SetAttribute ("Mode", StringValue ("UseLocal"));
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-left"));
  tapBridge.Install (nodes.Get (0), csma_left_devices.Get (0));
  // right container
  tapBridge.SetAttribute ("Mode", StringValue ("UseLocal"));
  tapBridge.SetAttribute ("DeviceName", StringValue ("tap-right"));
  tapBridge.Install (nodes.Get (1), csma_right_devices.Get (0));
  
  TapBridgeHelper tapBridge1;
  tapBridge1.SetAttribute ("Mode", StringValue ("UseLocal"));
  tapBridge1.SetAttribute ("DeviceName", StringValue ("tap-left-1"));
  tapBridge1.Install (nodes.Get (0), wifiDevices.Get (0));
  tapBridge1.SetAttribute ("DeviceName", StringValue ("tap-right-1"));
  tapBridge1.Install (nodes.Get (1), csma_AP_right_devices.Get (1));

  
   // ****************************************************************************************************************
  //                        Configure ROUTING
  // ****************************************************************************************************************
  
  //  UE routing 
  // Set the default gateway for the UE using a static routing
  Ptr<Ipv4StaticRouting> ueStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (ueNode.Get (0)->GetObject<Ipv4> ());
  ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

  // PGW 
  // Adding the network behind the UE to the pgw --> hardcoding the IP address of the UE connected to the external network
  Ptr<Ipv4StaticRouting> pgwStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (pgw->GetObject<Ipv4> ());
  pgwStaticRouting->AddNetworkRouteTo (Ipv4Address ("11.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                       Ipv4Address ("7.0.0.2"), 1);
  // pgwStaticRouting->AddNetworkRouteTo (Ipv4Address ("15.0.0.0"), Ipv4Mask ("255.0.0.0"),
  //                                      Ipv4Address ("7.0.0.2"), 1);

  // // add route to wifi interface of left node from UE
  // Ptr<Ipv4StaticRouting> ueStaticRouting =
  //     ipv4RoutingHelper.GetStaticRouting (ueNode.Get(0)->GetObject<Ipv4> ());
  // ueStaticRouting->AddNetworkRouteTo (Ipv4Address ("15.0.0.0"), Ipv4Mask ("255.0.0.0"),
  //                                     Ipv4Address ("11.0.0.2"), 2);


  
  
  // ********************************************************
  // turn off AP csma interface after N sec
  // turn ON AP csma interface after M sec
  // ********************************************************
 
  // Ptr<Ipv4> ipv4_AP = wifiAP.Get(0)->GetObject<Ipv4> ();
  // // std::cout << "index " << csmaAPinface.Get(0).second << std::endl; 
  // uint32_t ipv4ifIndex = 2;   // index of csma interface of AP 
  
  //ipv4_AP->SetDown(2); 

  // Simulator::Schedule (Seconds (15), &Ipv4::SetDown, ipv4_AP, ipv4ifIndex);
  // Simulator::Schedule (Seconds (25), &Ipv4::SetUp, ipv4_AP, ipv4ifIndex);

  // ********************************************************
  // Debug: Testing that proper IP addresses are configured
  // ********************************************************
  Ptr<Node> ueNodeZero = ueNode.Get (0);
  Ipv4Address gateway = epcHelper->GetUeDefaultGatewayAddress ();
  Ptr<Ipv4> ipv4_ue = ueNodeZero->GetObject<Ipv4> ();
  Ipv4Address addr1_ue = ipv4_ue->GetAddress (1, 0).GetLocal ();
  std::cout << "UE LTE ip address is: " << addr1_ue << std::endl;
  std::cout << "UE default gateway is: " << gateway << std::endl;
  Ipv4Address addr2_ue = ipv4_ue->GetAddress (2, 0).GetLocal ();
  std::cout << "UE csma IP address is: " << addr2_ue << std::endl;

  Ptr<Ipv4> ipv4_pgw = pgw->GetObject<Ipv4> (); 
  Ipv4Address addr1_pgw = ipv4_pgw->GetAddress (1, 0).GetLocal ();
  std::cout << "PGW IP@ on ifc 1:: " << addr1_pgw << std::endl;
  Ipv4Address addr2_pgw = ipv4_pgw->GetAddress (2, 0).GetLocal ();
  std::cout << "PGW IP@ on ifc 2: " << addr2_pgw << std::endl;
  Ipv4Address addr3_pgw = ipv4_pgw->GetAddress (3, 0).GetLocal ();
  std::cout << "PGW IP@ on ifc 3: " << addr3_pgw << std::endl;

  // LTE statistics
  // lteHelper->EnableTraces ();

  // LTE connection to ghost nodes
  // csma_right.EnablePcap ("lena-csma-right", csma_right_devices.Get (0), true);
  //csma.EnablePcap ("lena-csma-right", csma_AP_right_devices.Get (0), true);

  csma.EnablePcapAll("csma_lte", true);
  //csma_right.EnablePcapAll("csma_right", true);

  // wifi ENABLE PCAP
  //wifiPhy.EnablePcapAll("mp-wifi-lte",true);
   
  // Flow monitor
  // Ptr<FlowMonitor> flowMonitor;
  // FlowMonitorHelper flowHelper;
  // flowMonitor = flowHelper.InstallAll();

  // scheduling throughput to be printed every 3 seconds
  //ThroughputMonitor (&flowHelper, flowMonitor);

  // print routing table of PGW
  std::cout << "routing table of PGW" << std::endl;
  Ptr<ns3::OutputStreamWrapper> strwrp = Create<OutputStreamWrapper> (&std::cout);
  pgwStaticRouting->PrintRoutingTable (strwrp);
  
  // // print routing table of UE
  // std::cout << "routing table of UE" << std::endl;
  // // Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodes.Get(0)->GetObject<Ipv4> ());
  // ueStaticRouting->PrintRoutingTable (strwrp);


  // stop simulation 
  Simulator::Stop (Seconds (simTime));


  // Animation
  // AnimationInterface anim ("mp-lte-animation.xml"); // Mandatory
  // anim.EnablePacketMetadata (); // Optional
  // anim.EnableIpv4RouteTracking ("routingtable-wireless.xml", Seconds (0), Seconds (5),
  //                               Seconds (0.25)); //Optional
  // anim.EnableIpv4L3ProtocolCounters (Seconds (1), Seconds (simTime));
  
  
  Simulator::Run ();

  /*GtkConfigStore config;
  config.ConfigureAttributes();*/

  Simulator::Destroy();

  //flowMonitor->SerializeToXmlFile("mp-flow-monitor.xml", true, true); 

  return 0;

}
