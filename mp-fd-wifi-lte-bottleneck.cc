/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/* *
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

#include "ns3/abort.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/csma-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/assert.h"
#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/lte-module.h"
#include "ns3/config-store.h"

#include <fcntl.h>  /* O_RDWR */
#include <string.h> /* memset(), memcpy() */
#include <stdio.h> /* perror(), printf(), fprintf() */
#include <stdlib.h> /* exit(), malloc(), free() */
#include <sys/ioctl.h> /* ioctl() */

/* includes for struct ifreq, etc */
#include <sys/types.h>
#include <sys/socket.h>
#include <linux/if.h>
#include <linux/if_tun.h>
#include <unistd.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("MpFdExample");

/**
   * \brief  we need file descriptor of previosly created TAP device to connect it to the FdNetDevice
   * by using a function SetFileDescriptor(int fd). this way we can send and receive L2 trafic to/from 
   * real world  
   * \param dev device name in string (ex. "tap0")
   * \return int value : file descriptor of TAP device 
*/
int tun_open( std::string dev)
{
 
  struct ifreq ifr;
  int fd, err;
  const std::string clonedev = "/dev/net/tun";

   /* open the clone device */
   if( (fd = open(clonedev.c_str(), O_RDWR)) < 0 ) {
     return fd;
   }

   /* preparation of the struct ifr, of type "struct ifreq" */
   memset(&ifr, 0, sizeof(ifr));

   NS_ASSERT_MSG (!dev.empty (), "TAP device name should be specifyed");
   if (!dev.empty ())
     {
       NS_ASSERT_MSG (dev.size () > IFNAMSIZ - 1, " TAP device name size is too big");
       strncpy (ifr.ifr_name, dev.c_str (), dev.size () + 1);
       /* TAP interface, No packet information */
       ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
     }

   /* try to create the device */
   if( (err = ioctl(fd, TUNSETIFF, (void *) &ifr)) < 0 ) {
     close(fd);
     perror("ioctl TUNSETIFF");close(fd);
     std::exit(1); 
     return err;
   }  
  
  std::cout << dev<< ":  FD = "<< fd << std::endl; 

  /* this is the special file descriptor that the caller will use to talk
   * with the virtual interface */
  return fd; 

}

// TODO: remove this function 
static void
PingRtt (std::string context, Time rtt)
{
  NS_LOG_UNCOND ("Received Response with RTT = " << rtt);
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

// ****************************************************************************************************************
// ****************************************************************************************************************
//                                      MAIN
// ****************************************************************************************************************
// ****************************************************************************************************************
int
main (int argc, char *argv[])
{
  NS_LOG_INFO ("Ping Emulation Example with TAP");

  std::string remote ("17.0.0.2"); 
  std::string mask ("255.0.0.0");
  std::string pi ("no");
  
  // tap devices name (should be pre created) 
  std::string tap_l ("tap-left");
  std::string tap_l1 ("tap-left-1");
  std::string tap_r ("tap-right");
  std::string tap_r1 ("tap-right-1");
  uint64_t path2delay = 1;            // delay between AP and remote host [ms]
  double simTime = 60;                // sim time, 1 min by default 
  //
  // Allow the user to override any of the defaults at run-time, via
  // command-line arguments
  //
  CommandLine cmd (__FILE__);
  cmd.AddValue ("remote", "Remote IP address (dotted decimal only please)", remote);
  cmd.AddValue ("tapMask", "Network mask for configure the TAP device (dotted decimal only please)", mask);
  cmd.AddValue ("modePi", "If 'yes' a PI header will be added to the traffic traversing the device(flag IFF_NOPI will be unset).", pi);
   cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue("path2delay", "delay between AP and remote host on second path [ms]", path2delay);
  cmd.Parse (argc, argv);

  // TODO:: remove or change 
  Ipv4Address remoteIp (remote.c_str ());
  Ipv4Mask tapMask (mask.c_str ());

  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  // ****************************************
  // Global configurations  
  // ****************************************
  // config output
  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));

  //
  // ****************************************
  // Nodes creation   
  // ****************************************
  // Create two ghost nodes.  The first will represent the virtual machine host
  // on the left side of the network; and the second will represent the VM on 
  // the right side.
  //
  NodeContainer nodes;          // Create two ghost nodes. 
  nodes.Create (2);

  NodeContainer nodeAP;         // create one wifi AP node
  nodeAP.Create(1);                   

  NodeContainer enbNode;       // create one eNB node     
  enbNode.Create (1);

  NodeContainer nodes_l_ap (nodes.Get(0), nodeAP.Get(0));
  NodeContainer nodes_r_ap (nodes.Get(1), nodeAP.Get(0));

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
  inet.Install (nodes); 

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

  // Install devices on nodes from path #1
  NetDeviceContainer dev_l_ap = wifi.Install (wifiPhy, wifiMac, nodes_l_ap);
  
  // Assign adress 
  ipv4h.SetBase ("16.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_ap_wifi = ipv4h.Assign (dev_l_ap.Get(1));
  Ipv4InterfaceContainer ifce_l_wifi = ipv4h.Assign (dev_l_ap.Get(0));
 

  // ****************************************************************************************************************
  //                                  Configure PATH 2: LTE and CSMA 
  // ****************************************************************************************************************
  
  // Default LTE configuration
  Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::RrFfMacScheduler::HarqEnabled", BooleanValue (false));
  Config::SetDefault ("ns3::LteHelper::PathlossModel",
                      StringValue ("ns3::FriisSpectrumPropagationLossModel"));
  Config::SetDefault ("ns3::LteEnbNetDevice::UlBandwidth", UintegerValue (100)); //20MHz bandwidth
  Config::SetDefault ("ns3::LteEnbNetDevice::DlBandwidth", UintegerValue (100)); //20MHz bandwidth
  Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010));

  ConfigStore outputConfig;
  outputConfig.ConfigureDefaults ();
  outputConfig.ConfigureAttributes ();

  // TODO:: change to LteHelper lteHelper instead of CreateObject and move to up

  // get LTE / EPC helpers  
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  // Get the pgw node to install the csma device
  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Install LTE Devices to the nodes
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNode);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (NodeContainer (nodes.Get(0)));  // TODO: test without NodeContainer

  // assign ipv4 adresses to UE Devices
  Ipv4InterfaceContainer ifce_l_lte = epcHelper->AssignUeIpv4Address (ueLteDevs);

  // Attach one UE per eNodeB
  // side effect: the default EPS bearer will be activated
  lteHelper->Attach (ueLteDevs.Get(0), enbLteDevs.Get (0));

  // Link: PGW <---> Remote node through CSMA 

  csma.SetChannelAttribute ("DataRate", DataRateValue (DataRate ("10Mb/s"))); 
  csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (1)));
  NodeContainer nodes_r_pgw (nodes.Get (1), pgw);
  NetDeviceContainer dev_r_pgw = csma.Install (nodes_r_pgw);

  // and assign ipv4 adress to ifaces  
  ipv4h.SetBase ("12.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifce_pgw_csma = ipv4h.Assign (NetDeviceContainer (dev_r_pgw.Get(1)));
  Ipv4InterfaceContainer ifce_r_csma1 = ipv4h.Assign (NetDeviceContainer (dev_r_pgw.Get(0)));


  // create link  pgw < -- > wifi AP
  // configure CSMA connection  
  NodeContainer apPgw (nodeAP.Get(0),pgw); 

  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));

  csma.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (path2delay)));

  // Install devices 
  NetDeviceContainer dev_ap_pgw = csma.Install (apPgw);
  
  // Assign adress 
  ipv4h.SetBase ("17.0.0.0", "255.0.0.0", "0.0.0.1");
  Ipv4InterfaceContainer ifces_ap_pgw = ipv4h.Assign (dev_ap_pgw);
  
  // ****************************************************************************************************************
  //                  Configure FDNetDevices and connect to TAP file descriptor 
  // ****************************************************************************************************************
  
  // Create an fd device, set a MAC address and point the device to the Linux device name.
  FdNetDeviceHelper fdNet;
  NetDeviceContainer tapDevs = fdNet.Install (nodes);
  NetDeviceContainer tapDevs1 = fdNet.Install (nodes);

  // lte path 
  Ptr<NetDevice> l0 = tapDevs.Get (0);
  Ptr<FdNetDevice> leftFdDev = l0->GetObject<FdNetDevice> ();
  leftFdDev->SetFileDescriptor (tun_open(tap_l));
  Ptr<NetDevice> r0= tapDevs.Get (1);
  Ptr<FdNetDevice> rightFdDev = r0->GetObject<FdNetDevice> ();
  rightFdDev->SetFileDescriptor (tun_open(tap_r));
  
  // wifi path 
  Ptr<NetDevice> l1 = tapDevs1.Get (0);
  Ptr<FdNetDevice> leftFdDev1 = l1->GetObject<FdNetDevice> ();
  leftFdDev1->SetFileDescriptor (tun_open(tap_l1));
  Ptr<NetDevice> r1= tapDevs1.Get (1);
  Ptr<FdNetDevice> rightFdDev1 = r1->GetObject<FdNetDevice> ();
  rightFdDev1->SetFileDescriptor (tun_open(tap_r1));

  // we have a FdNetDevices connected to TAP devices through fd, 
  // now install it on nodes and set IP addresses
  Ptr<Ipv4> ipv4;                 // get Ipv4 object from node 
  Ipv4InterfaceAddress address;   // interface address 
  uint32_t interface;             // ifce number 
  
  // left node (UE)
  ipv4 = nodes.Get(0)->GetObject<Ipv4> ();  
  interface = ipv4->AddInterface (leftFdDev1);
  address = Ipv4InterfaceAddress (Ipv4Address ("15.0.0.1"), Ipv4Mask("255.0.0.0"));
  std::cout<<"Adress 1: " << address << std::endl; 
  ipv4->AddAddress (interface, address);             
  // ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  // left node (UE)
  ipv4 = nodes.Get(0)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (leftFdDev);
  address = Ipv4InterfaceAddress (Ipv4Address ("11.0.0.1"), Ipv4Mask("255.0.0.0"));
  std::cout<<"Adress 1: " << address << std::endl; 
  ipv4->AddAddress (interface, address);              
  // ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  // right node (Remote)
  ipv4 = nodes.Get(1)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (rightFdDev1);
  address = Ipv4InterfaceAddress (Ipv4Address ("14.0.0.1"), Ipv4Mask("255.0.0.0"));
  std::cout<<"Adress 2: " << address << std::endl; 
  ipv4->AddAddress (interface, address);
  // ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  // right node (Remote)
  ipv4 = nodes.Get(1)->GetObject<Ipv4> ();
  interface = ipv4->AddInterface (rightFdDev);
  address = Ipv4InterfaceAddress (Ipv4Address ("13.0.0.1"), Ipv4Mask("255.0.0.0"));
  std::cout<<"Adress 2: " << address << std::endl; 
  ipv4->AddAddress (interface, address);
  // ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  // ****************************************************************************************************************
  //                        Configure ROUTING
  // ****************************************************************************************************************

  //Adding the network behind the UE and remote host to the AP routing table
  // hardcoding the IP address of the LXC interfaces connected to this path
  // TODO :::: change routing

  // *********************************
  // routing on AP node 
  // *********************************
  Ptr<Ipv4StaticRouting> apStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (nodeAP.Get (0)->GetObject<Ipv4> ());
  apStaticRouting->AddNetworkRouteTo (Ipv4Address ("15.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                      Ipv4Address ("16.0.0.2"), ifce_ap_wifi.Get (0).second);
  apStaticRouting->AddNetworkRouteTo (Ipv4Address ("14.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                      Ipv4Address ("17.0.0.2"), ifces_ap_pgw.Get(0).second);

  // ******************************** 
  // routing on left node (UE)                
  // ********************************
  Ptr<Ipv4StaticRouting> ueStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (nodes.Get (0)->GetObject<Ipv4> ());
  // Set the default gateway for the UE using a static routing
  ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (),
                                    ifce_l_lte.Get(0).second);

  // add route to right lxc through wifi
  ueStaticRouting->AddNetworkRouteTo (Ipv4Address ("14.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                      Ipv4Address ("16.0.0.1"), ifce_l_wifi.Get (0).second);

  // ********************************
  // routing on right node (Remote)
  // ********************************
  Ptr<Ipv4StaticRouting> rightStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (nodes.Get (1)->GetObject<Ipv4> ());
  // default route through LTE path
  rightStaticRouting->SetDefaultRoute (Ipv4Address ("12.0.0.1"), ifce_r_csma1.Get (0).second);
 
  // ********************************
  // routing on PGW node
  // ********************************
  // Adding the network behind the UE to the pgw
  // hardcoding the IP address of the UE connected to the external network
  Ptr<Ipv4StaticRouting> pgwStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (pgw->GetObject<Ipv4> ());
  pgwStaticRouting->AddNetworkRouteTo (Ipv4Address ("11.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                       Ipv4Address ("7.0.0.2"), 1);
  pgwStaticRouting->AddNetworkRouteTo (Ipv4Address ("13.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                       Ipv4Address ("12.0.0.2"), ifce_pgw_csma.Get(0).second);
  pgwStaticRouting->AddNetworkRouteTo (Ipv4Address ("15.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                       Ipv4Address ("17.0.0.1"), ifces_ap_pgw.Get(1).second);
  pgwStaticRouting->AddNetworkRouteTo (Ipv4Address ("14.0.0.0"), Ipv4Mask ("255.0.0.0"),
                                       Ipv4Address ("12.0.0.2"), ifce_pgw_csma.Get(0).second);

  // ADD lxc mp-left address to allow "ping" through EPC-PGW node
  addBackAddress (pgw, ueLteDevs.Get (0), Ipv4Address ("11.0.0.2"));
  
  // ****************************************
  // set possition and mobility model
  // ****************************************
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0, 0, 0));    // pos of left
  positionAlloc->Add (Vector (5, 0, 0));    // pos of AP
  positionAlloc->Add (Vector (25, 0, 0));   // pos of right
  positionAlloc->Add (Vector (30, 0, 0));   // pos of eNodeB

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes.Get (0));
  mobility.Install (nodeAP.Get (0));
  mobility.Install (nodes.Get(1));
  mobility.Install (enbNode.Get(0));

  // ****************************************************************************************************************
  //                        Configure APPLICATIONS 
  // ****************************************************************************************************************
  //
  // Create the ping application.  This application knows how to send
  // ICMP echo requests.  Setting up the packet sink manually is a bit
  // of a hassle and since there is no law that says we cannot mix the
  // helper API with the low level API, let's just use the helper.
  //
  NS_LOG_INFO ("Create V4Ping Appliation");
  Ptr<V4Ping> app = CreateObject<V4Ping> ();
  app->SetAttribute ("Remote", Ipv4AddressValue (remoteIp));
  app->SetAttribute ("Verbose", BooleanValue (true));
  nodes.Get(0)->AddApplication (app);
  app->SetStartTime (Seconds (1.0));
  app->SetStopTime (Seconds (11.0));
  
  //
  // Give the application a name.  This makes life much easier when constructing
  // config paths.
  //
  Names::Add ("app", app);

  //
  // Hook a trace to print something when the response comes back.
  //
  Config::Connect ("/Names/app/Rtt", MakeCallback (&PingRtt));

  wifiPhy.EnablePcapAll ("mp-wifi", true);
  csma.EnablePcapAll("mp-csma", true);
  fdNet.EnablePcapAll("mp-fd",true);  


  // ********************************************************
  // Debug: Testing that proper IP addresses are configured
  // ********************************************************
  // Ptr<Node> ueNodeZero = nodes.Get (0);
  // Ipv4Address gateway = epcHelper->GetUeDefaultGatewayAddress ();
  // Ptr<Ipv4> ipv4_ue = ueNodeZero->GetObject<Ipv4> ();
  // Ipv4Address addr1_ue = ipv4_ue->GetAddress (2, 0).GetLocal ();
  // std::cout << "UE LTE ip address is: " << addr1_ue << std::endl;
  // std::cout << "UE default gateway is: " << gateway << std::endl;
  // Ipv4Address addr2_ue = ipv4_ue->GetAddress (1, 0).GetLocal ();
  // std::cout << "UE csma IP address is: " << addr2_ue << std::endl;

  // Ptr<Ipv4> ipv4_pgw = pgw->GetObject<Ipv4> (); 
  // Ipv4Address addr1_pgw = ipv4_pgw->GetAddress (1, 0).GetLocal ();
  // std::cout << "PGW IP@ on ifc 1:: " << addr1_pgw << std::endl;
  // Ipv4Address addr2_pgw = ipv4_pgw->GetAddress (2, 0).GetLocal ();
  // std::cout << "PGW IP@ on ifc 2: " << addr2_pgw << std::endl;
  // Ipv4Address addr3_pgw = ipv4_pgw->GetAddress (3, 0).GetLocal ();
  // std::cout << "PGW IP@ on ifc 3: " << addr3_pgw << std::endl;

  //
  // Now, do the actual emulation.
  //
  NS_LOG_INFO ("Run Emulation.");
  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
  NS_LOG_INFO ("Done.");
}

