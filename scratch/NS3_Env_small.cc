/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/mygym.h"
// #include "ns3/netanim-module.h"
#include <iostream>
#include <vector>
#include <stdio.h>
#include <iomanip>
#include "ns3/netanim-module.h"
#include "ns3/building.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/buildings-channel-condition-model.h"
#include "ns3/buildings-module.h"
#include "ns3/buildings-helper.h"
#include "ns3/buildings-propagation-loss-model.h"



using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LenaX2HandoverMeasures");
 
 void
 SetVelocity(Ptr<ConstantVelocityMobilityModel> node, Vector vel){
      node->SetVelocity(vel);
 }        
 

void
NotifyConnectionEstablishedUe (std::string context,
                               uint64_t imsi,
                               uint16_t cellid,
                               uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": connected to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": previously connected to CellId " << cellid
            << " with RNTI " << rnti
            << ", doing handover to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkUe (std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti)
{
  std::cout << context
            << " UE IMSI " << imsi
            << ": successful handover to CellId " << cellid
            << " with RNTI " << rnti
            << std::endl;
}

void
NotifyConnectionEstablishedEnb (std::string context,
                                uint64_t imsi,
                                uint16_t cellid,
                                uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": successful connection of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

void
NotifyHandoverStartEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti,
                        uint16_t targetCellId)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": start handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << " to CellId " << targetCellId
            << std::endl;
}

void
NotifyHandoverEndOkEnb (std::string context,
                        uint64_t imsi,
                        uint16_t cellid,
                        uint16_t rnti)
{
  std::cout << context
            << " eNB CellId " << cellid
            << ": completed handover of UE with IMSI " << imsi
            << " RNTI " << rnti
            << std::endl;
}

bool
AreOverlapping (Box a, Box b)
{
  return !((a.xMin > b.xMax) || (b.xMin > a.xMax) || (a.yMin > b.yMax) || (b.yMin > a.yMax) );
}


bool
OverlapWithAnyPrevious (Box box, std::list<Box> m_previousBlocks)
{
  for (std::list<Box>::iterator it = m_previousBlocks.begin (); it != m_previousBlocks.end (); ++it)
    {
      if (AreOverlapping (*it,box))
        {
          return true;
        }
    }
  return false;
}


std::pair<Box, std::list<Box> >
GenerateBuildingBounds (double xmin, double xmax, double ymin, double ymax, double maxBuildSize, std::list<Box> m_previousBlocks )
{

  Ptr<UniformRandomVariable> xMinBuilding = CreateObject<UniformRandomVariable> ();
  xMinBuilding->SetAttribute ("Min",DoubleValue (xmin));
  xMinBuilding->SetAttribute ("Max",DoubleValue (xmax));

  NS_LOG_UNCOND ("min " << xmin << " max " << xmax);

  Ptr<UniformRandomVariable> yMinBuilding = CreateObject<UniformRandomVariable> ();
  yMinBuilding->SetAttribute ("Min",DoubleValue (ymin));
  yMinBuilding->SetAttribute ("Max",DoubleValue (ymax));

  NS_LOG_UNCOND ("min " << ymin << " max " << ymax);

  Box box;
  uint32_t attempt = 0;
  do
    {
      NS_ASSERT_MSG (attempt < 100, "Too many failed attempts to position non-overlapping buildings. Maybe area too small or too many buildings?");
      box.xMin = xMinBuilding->GetValue ();

      Ptr<UniformRandomVariable> xMaxBuilding = CreateObject<UniformRandomVariable> ();
      xMaxBuilding->SetAttribute ("Min",DoubleValue (box.xMin));
      xMaxBuilding->SetAttribute ("Max",DoubleValue (box.xMin + maxBuildSize));
      box.xMax = xMaxBuilding->GetValue ();

      box.yMin = yMinBuilding->GetValue ();

      Ptr<UniformRandomVariable> yMaxBuilding = CreateObject<UniformRandomVariable> ();
      yMaxBuilding->SetAttribute ("Min",DoubleValue (box.yMin));
      yMaxBuilding->SetAttribute ("Max",DoubleValue (box.yMin + maxBuildSize));
      box.yMax = yMaxBuilding->GetValue ();

      ++attempt;
    }
  while (OverlapWithAnyPrevious (box, m_previousBlocks));


  NS_LOG_UNCOND ("Building in coordinates (" << box.xMin << " , " << box.yMin << ") and ("  << box.xMax << " , " << box.yMax <<
                 ") accepted after " << attempt << " attempts");
  m_previousBlocks.push_back (box);
  std::pair<Box, std::list<Box> > pairReturn = std::make_pair (box,m_previousBlocks);
  return pairReturn;

}
                              
/**
 * Sample simulation script for an automatic X2-based handover based on the RSRQ measures.
 * It instantiates two eNodeB, attaches one UE to the 'source' eNB.
 * The UE moves between both eNBs, it reports measures to the serving eNB and
 * the 'source' (serving) eNB triggers the handover of the UE towards
 * the 'target' eNB when it considers it is a better eNB.
 */
uint32_t RunNum;

int
main (int argc, char *argv[])
{
  //LogLevel logLevel = (LogLevel)(LOG_PREFIX_ALL | LOG_LEVEL_ALL);

  // LogComponentEnable ("LteHelper", logLevel);
  // LogComponentEnable ("EpcHelper", logLevel);
  // LogComponentEnable ("EpcEnbApplication", logLevel);
  // LogComponentEnable ("EpcMmeApplication", logLevel);
  // LogComponentEnable ("EpcPgwApplication", logLevel);
  // LogComponentEnable ("EpcSgwApplication", logLevel);
  // LogComponentEnable ("EpcX2", logLevel);

  // LogComponentEnable ("RrFfMacScheduler", logLevel);
  // LogComponentEnable ("LteEnbRrc", logLevel);
  // LogComponentEnable ("LteEnbNetDevice", logLevel);
  // LogComponentEnable ("LteUeRrc", logLevel);
  // LogComponentEnable ("LteUeNetDevice", logLevel);
  // LogComponentEnable ("A2A4RsrqHandoverAlgorithm", logLevel);
  // LogComponentEnable ("A3RsrpHandoverAlgorithm", logLevel);

  uint16_t numberOfUes = 40;
  uint16_t numberOfEnbs = 5;
  uint16_t numBearersPerUe = 1;
  bool disableDl = false;
  bool disableUl = false;
  bool enablebuilding = true;
  uint16_t numBlocks = 2;
  std::list<Box>  m_previousBlocks;
  double speed = 20;       // m/s
  double simTime = 15;
  double enbTxPowerDbm = 46.0;
  double steptime = 0.5;
  uint16_t macroEnbBandwidth = 75;

  //opengym environment
  uint32_t openGymPort = 1403;

  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (10)));
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (100000));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (true));

  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("simTime", "Total duration of the simulation (in seconds)", simTime);
  cmd.AddValue ("speed", "Speed of the UE (default = 20 m/s)", speed);
  cmd.AddValue ("enbTxPowerDbm", "TX power [dBm] used by HeNBs (default = 46.0)", enbTxPowerDbm);
  cmd.AddValue ("RunNum" , "1...10" , RunNum);

  cmd.Parse (argc, argv);

  
  //Building Creation and Attribute Settings
  if(enablebuilding){
  double maxBuildingSize = 4;

  std::vector<Ptr<Building> > buildingVector1;

  for (uint32_t buildingIndex = 0; buildingIndex < numBlocks; buildingIndex++)
    {
      Ptr < Building > building;
      building = Create<Building> ();
      /* returns a vecotr where:
      * position [0]: coordinates for x min
      * position [1]: coordinates for x max
      * position [2]: coordinates for y min
      * position [3]: coordinates for y max
      */
      std::pair<Box, std::list<Box> > pairBuildings = GenerateBuildingBounds (175, 225, 20, 30, maxBuildingSize, m_previousBlocks);
      m_previousBlocks = std::get<1> (pairBuildings);
      Box box = std::get<0> (pairBuildings);
      Ptr<UniformRandomVariable> randomBuildingZ = CreateObject<UniformRandomVariable> ();
      randomBuildingZ->SetAttribute ("Min",DoubleValue (1.6));
      randomBuildingZ->SetAttribute ("Max",DoubleValue (40));
      double buildingHeight = randomBuildingZ->GetValue ();

      building->SetBoundaries (Box (box.xMin, box.xMax,
                                    box.yMin,  box.yMax,
                                    0.0, buildingHeight));
      buildingVector1.push_back (building);
    }

  std::vector<Ptr<Building> > buildingVector2;

  for (uint32_t buildingIndex = 0; buildingIndex < numBlocks; buildingIndex++)
    {
      Ptr < Building > building;
      building = Create<Building> ();
      /* returns a vecotr where:
      * position [0]: coordinates for x min
      * position [1]: coordinates for x max
      * position [2]: coordinates for y min
      * position [3]: coordinates for y max
      */
      std::pair<Box, std::list<Box> > pairBuildings = GenerateBuildingBounds (175, 225, 370, 380, maxBuildingSize, m_previousBlocks);
      m_previousBlocks = std::get<1> (pairBuildings);
      Box box = std::get<0> (pairBuildings);
      Ptr<UniformRandomVariable> randomBuildingZ = CreateObject<UniformRandomVariable> ();
      randomBuildingZ->SetAttribute ("Min",DoubleValue (1.6));
      randomBuildingZ->SetAttribute ("Max",DoubleValue (40));
      double buildingHeight = randomBuildingZ->GetValue ();

      building->SetBoundaries (Box (box.xMin, box.xMax,
                                    box.yMin,  box.yMax,
                                    0.0, buildingHeight));
      buildingVector2.push_back (building);
    }
    
    std::vector<Ptr<Building> > buildingVector3;

  for (uint32_t buildingIndex = 0; buildingIndex < numBlocks; buildingIndex++)
    {
      Ptr < Building > building;
      building = Create<Building> ();
      /* returns a vecotr where:
      * position [0]: coordinates for x min
      * position [1]: coordinates for x max
      * position [2]: coordinates for y min
      * position [3]: coordinates for y max
      */
      std::pair<Box, std::list<Box> > pairBuildings = GenerateBuildingBounds (20, 30, 175, 225, maxBuildingSize, m_previousBlocks);
      m_previousBlocks = std::get<1> (pairBuildings);
      Box box = std::get<0> (pairBuildings);
      Ptr<UniformRandomVariable> randomBuildingZ = CreateObject<UniformRandomVariable> ();
      randomBuildingZ->SetAttribute ("Min",DoubleValue (1.6));
      randomBuildingZ->SetAttribute ("Max",DoubleValue (40));
      double buildingHeight = randomBuildingZ->GetValue ();

      building->SetBoundaries (Box (box.xMin, box.xMax,
                                    box.yMin,  box.yMax,
                                    0.0, buildingHeight));
      buildingVector3.push_back (building);
    }

    std::vector<Ptr<Building> > buildingVector4;

  for (uint32_t buildingIndex = 0; buildingIndex < numBlocks; buildingIndex++)
    {
      Ptr < Building > building;
      building = Create<Building> ();
      /* returns a vecotr where:
      * position [0]: coordinates for x min
      * position [1]: coordinates for x max
      * position [2]: coordinates for y min
      * position [3]: coordinates for y max
      */
      std::pair<Box, std::list<Box> > pairBuildings = GenerateBuildingBounds (370, 380, 175, 225, maxBuildingSize, m_previousBlocks);
      m_previousBlocks = std::get<1> (pairBuildings);
      Box box = std::get<0> (pairBuildings);
      Ptr<UniformRandomVariable> randomBuildingZ = CreateObject<UniformRandomVariable> ();
      randomBuildingZ->SetAttribute ("Min",DoubleValue (1.6));
      randomBuildingZ->SetAttribute ("Max",DoubleValue (40));
      double buildingHeight = randomBuildingZ->GetValue ();

      building->SetBoundaries (Box (box.xMin, box.xMax,
                                    box.yMin,  box.yMax,
                                    0.0, buildingHeight));
      buildingVector4.push_back (building);
    }
  }
  
    


  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
   
      // lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::MmWavePropagationLossModel"));
     if(enablebuilding) {
      lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ThreeGppV2vHighwayPropagationLossModel"));
      lteHelper->SetChannelConditionModelType ("ns3::ThreeGppV2vHighwayChannelConditionModel");
     }

     
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  // lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
  lteHelper->SetSchedulerType ("ns3::PfFfMacScheduler");

  //Test (eNB badwidth)
  lteHelper -> SetEnbDeviceAttribute("DlBandwidth", UintegerValue(macroEnbBandwidth));
  lteHelper -> SetEnbDeviceAttribute("UlBandwidth", UintegerValue(macroEnbBandwidth));

   lteHelper->SetHandoverAlgorithmType ("ns3::A3RsrpHandoverAlgorithm");
   lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis",
                                             DoubleValue (0.0));
   lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger",
                                             TimeValue (MilliSeconds (0)));

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Mb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);
  // To Check Remote Host's Address
  // std::cout << "Remote Host Address" << internetIpIfaces.GetAddress(1);


  // Routing of the Internet Host (towards the LTE network)
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  // interface 0 is localhost, 1 is the p2p device
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create (numberOfEnbs);
  ueNodes.Create (numberOfUes);
  
  
   MobilityHelper enbMobility;
  enbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  enbMobility.Install (enbNodes);
  Ptr<ConstantPositionMobilityModel> enb0 = DynamicCast<ConstantPositionMobilityModel>(enbNodes.Get(0)->GetObject<MobilityModel> ());	
  Ptr<ConstantPositionMobilityModel> enb1 = DynamicCast<ConstantPositionMobilityModel>(enbNodes.Get(1)->GetObject<MobilityModel> ());	 
  Ptr<ConstantPositionMobilityModel> enb2 = DynamicCast<ConstantPositionMobilityModel>(enbNodes.Get(2)->GetObject<MobilityModel> ());	
  Ptr<ConstantPositionMobilityModel> enb3 = DynamicCast<ConstantPositionMobilityModel>(enbNodes.Get(3)->GetObject<MobilityModel> ());	
  Ptr<ConstantPositionMobilityModel> enb4 = DynamicCast<ConstantPositionMobilityModel>(enbNodes.Get(4)->GetObject<MobilityModel> ());	


	
  enb0->SetPosition(Vector(200,0,1.5));
  enb1->SetPosition(Vector(0,200,1.5)); 
  enb2->SetPosition(Vector(200,200,1.5));
  enb3->SetPosition(Vector(400,200,1.5));
  enb4->SetPosition(Vector(200,400,1.5));
  BuildingsHelper::Install (enbNodes);

  //////////////////////////////////////////////////////////////////////////////////////////////////////
  //  MobilityHelper ueMobility2_1;
  Ptr<RandomRectanglePositionAllocator> allocator2 = CreateObject<RandomRectanglePositionAllocator> ();
  Ptr<UniformRandomVariable> xPos2 = CreateObject<UniformRandomVariable> ();
  xPos2->SetAttribute ("Min", DoubleValue (150));
  xPos2->SetAttribute ("Max", DoubleValue (250));
  allocator2->SetX (xPos2);
  Ptr<UniformRandomVariable> yPos2 = CreateObject<UniformRandomVariable> ();
  yPos2->SetAttribute ("Min", DoubleValue (80.0));
  yPos2->SetAttribute ("Max", DoubleValue (120.0));
  allocator2->SetY (yPos2);
  allocator2->SetZ (1.5);
  allocator2->AssignStreams (1);

  MobilityHelper ueMobility2_2;
  ueMobility2_2.SetPositionAllocator (allocator2);
  ueMobility2_2.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 85, 110)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=25]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  // for (int i= 3 ; i < 6 ; i++)
  ueMobility2_2.Install (ueNodes.Get(1));

  MobilityHelper ueMobility2_3;
  ueMobility2_3.SetPositionAllocator (allocator2);
  ueMobility2_3.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 85, 110)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=30]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility2_3.Install (ueNodes.Get(2));

  MobilityHelper ueMobility2_4;
  ueMobility2_4.SetPositionAllocator (allocator2);
  ueMobility2_4.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 85, 110)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=35]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  // for (int i= 3 ; i < 6 ; i++)
  ueMobility2_4.Install (ueNodes.Get(3));

  MobilityHelper ueMobility2_5;
  ueMobility2_5.SetPositionAllocator (allocator2);
  ueMobility2_5.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 85, 110)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=40]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility2_5.Install (ueNodes.Get(4));

  MobilityHelper ueMobility2_6;
  ueMobility2_6.SetPositionAllocator (allocator2);
  ueMobility2_6.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 85, 110)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=45]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility2_6.Install (ueNodes.Get(5));

  MobilityHelper ueMobility2_7;
  ueMobility2_7.SetPositionAllocator (allocator2);
  ueMobility2_7.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 85, 110)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=50]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility2_7.Install (ueNodes.Get(6));

  MobilityHelper ueMobility2_8;
  ueMobility2_8.SetPositionAllocator (allocator2);
  ueMobility2_8.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 85, 110)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=55]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility2_8.Install (ueNodes.Get(8));

  MobilityHelper ueMobility2_9;
  ueMobility2_9.SetPositionAllocator (allocator2);
  ueMobility2_9.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 85, 110)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=60]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility2_9.Install (ueNodes.Get(9));

//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////

  // MobilityHelper ueMobility3_1;
  Ptr<RandomRectanglePositionAllocator> allocator3 = CreateObject<RandomRectanglePositionAllocator> ();
  Ptr<UniformRandomVariable> xPos3 = CreateObject<UniformRandomVariable> ();
  xPos3->SetAttribute ("Min", DoubleValue (150.0));
  xPos3->SetAttribute ("Max", DoubleValue (250.0));
  allocator3->SetX (xPos3);
  Ptr<UniformRandomVariable> yPos3 = CreateObject<UniformRandomVariable> ();
  yPos3->SetAttribute ("Min", DoubleValue (280.0));
  yPos3->SetAttribute ("Max", DoubleValue (320.0));
  allocator3->SetY (yPos3);
  allocator3->SetZ (1.5);
  allocator3->AssignStreams (1);

  MobilityHelper ueMobility3_2;
  ueMobility3_2.SetPositionAllocator (allocator3);
  ueMobility3_2.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 290, 315)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=25]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility3_2.Install (ueNodes.Get(11));

  MobilityHelper ueMobility3_3;
  ueMobility3_3.SetPositionAllocator (allocator3);
  ueMobility3_3.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 290, 315)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=30]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility3_3.Install (ueNodes.Get(12));

  MobilityHelper ueMobility3_4;
  ueMobility3_4.SetPositionAllocator (allocator3);
  ueMobility3_4.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 290, 315)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=35]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility3_4.Install (ueNodes.Get(13));

  MobilityHelper ueMobility3_5;
  ueMobility3_5.SetPositionAllocator (allocator3);
  ueMobility3_5.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 290, 315)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=40]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility3_5.Install (ueNodes.Get(14));
  
  MobilityHelper ueMobility3_6;
  ueMobility3_6.SetPositionAllocator (allocator3);
  ueMobility3_6.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 290, 315)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=45]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility3_6.Install (ueNodes.Get(15));

  MobilityHelper ueMobility3_7;
  ueMobility3_7.SetPositionAllocator (allocator3);
  ueMobility3_7.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 290, 315)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=50]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility3_7.Install (ueNodes.Get(16));

  MobilityHelper ueMobility3_8;
  ueMobility3_8.SetPositionAllocator (allocator3);
  ueMobility3_8.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 290, 315)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=55]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility3_8.Install (ueNodes.Get(18));

  MobilityHelper ueMobility3_9;
  ueMobility3_9.SetPositionAllocator (allocator3);
  ueMobility3_9.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (150, 250, 290, 315)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=60]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility3_9.Install (ueNodes.Get(19));
  
//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////

  MobilityHelper ueMobility4_1;
  Ptr<RandomRectanglePositionAllocator> allocator4 = CreateObject<RandomRectanglePositionAllocator> ();
  Ptr<UniformRandomVariable> xPos4 = CreateObject<UniformRandomVariable> ();
  xPos4->SetAttribute ("Min", DoubleValue (280));
  xPos4->SetAttribute ("Max", DoubleValue (320));
  allocator4->SetX (xPos4);
  Ptr<UniformRandomVariable> yPos4 = CreateObject<UniformRandomVariable> ();
  yPos4->SetAttribute ("Min", DoubleValue (150));
  yPos4->SetAttribute ("Max", DoubleValue (250));
  allocator4->SetY (yPos4);
  allocator4->SetZ (1.5);
  allocator4->AssignStreams (1);
  ueMobility4_1.SetPositionAllocator (allocator4);
  ueMobility4_1.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (290, 315, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=20]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility4_1.Install (ueNodes.Get(20));

  MobilityHelper ueMobility4_2;
  ueMobility4_2.SetPositionAllocator (allocator4);
  ueMobility4_2.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (290, 315, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=25]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility4_2.Install (ueNodes.Get(21));

  MobilityHelper ueMobility4_3;
  ueMobility4_3.SetPositionAllocator (allocator4);
  ueMobility4_3.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (290, 315, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=30]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility4_3.Install (ueNodes.Get(22));

  MobilityHelper ueMobility4_4;
  ueMobility4_4.SetPositionAllocator (allocator4);
  ueMobility4_4.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (290, 315, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=35]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility4_4.Install (ueNodes.Get(23));

  MobilityHelper ueMobility4_5;
  ueMobility4_5.SetPositionAllocator (allocator4);
  ueMobility4_5.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (290, 315, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=40]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility4_5.Install (ueNodes.Get(24));

  MobilityHelper ueMobility4_6;
  ueMobility4_6.SetPositionAllocator (allocator4);
  ueMobility4_6.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (290, 315, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=45]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility4_6.Install (ueNodes.Get(25));

  MobilityHelper ueMobility4_7;
  ueMobility4_7.SetPositionAllocator (allocator4);
  ueMobility4_7.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (290, 315, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=50]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility4_7.Install (ueNodes.Get(26));

  MobilityHelper ueMobility4_7_1;
  ueMobility4_7_1.SetPositionAllocator (allocator4);
  ueMobility4_7_1.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (290, 315, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=15]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility4_7_1.Install (ueNodes.Get(27));

//////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////

   MobilityHelper ueMobility5_1;
  Ptr<RandomRectanglePositionAllocator> allocator5 = CreateObject<RandomRectanglePositionAllocator> ();
  Ptr<UniformRandomVariable> xPos5 = CreateObject<UniformRandomVariable> ();
  xPos5->SetAttribute ("Min", DoubleValue (80.0));
  xPos5->SetAttribute ("Max", DoubleValue (120.0));
  allocator5->SetX (xPos5);
  Ptr<UniformRandomVariable> yPos5 = CreateObject<UniformRandomVariable> ();
  yPos5->SetAttribute ("Min", DoubleValue (150.0));
  yPos5->SetAttribute ("Max", DoubleValue (250.0));
  allocator5->SetY (yPos5);
  allocator5->SetZ (1.5);
  allocator5->AssignStreams (1);
  ueMobility5_1.SetPositionAllocator (allocator5);
  ueMobility5_1.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (85, 110, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=20]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility5_1.Install (ueNodes.Get(30));

  MobilityHelper ueMobility5_2;
  ueMobility5_2.SetPositionAllocator (allocator5);
  ueMobility5_2.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (85, 110, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=25]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility5_2.Install (ueNodes.Get(31));

  MobilityHelper ueMobility5_3;
  ueMobility5_3.SetPositionAllocator (allocator5);
  ueMobility5_3.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (85, 110, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=30]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility5_3.Install (ueNodes.Get(32));

  MobilityHelper ueMobility5_4;
  ueMobility5_4.SetPositionAllocator (allocator5);
  ueMobility5_4.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (85, 110, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=35]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility5_4.Install (ueNodes.Get(33));

  MobilityHelper ueMobility5_5;
  ueMobility5_5.SetPositionAllocator (allocator5);
  ueMobility5_5.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (85, 110, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=40]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility5_5.Install (ueNodes.Get(34));

  MobilityHelper ueMobility5_6;
  ueMobility5_6.SetPositionAllocator (allocator5);
  ueMobility5_6.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (85, 110, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=45]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility5_6.Install (ueNodes.Get(35));

  MobilityHelper ueMobility5_7;
  ueMobility5_7.SetPositionAllocator (allocator5);
  ueMobility5_7.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (85, 110, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=50]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility5_7.Install (ueNodes.Get(36));

  MobilityHelper ueMobility5_7_1;
  ueMobility5_7_1.SetPositionAllocator (allocator5);
  ueMobility5_7_1.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (85, 110, 150, 250)),
                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=15]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  ueMobility5_7_1.Install (ueNodes.Get(37));

  //////////////////////////////////////////////////////////////////////////////////////////////////////

  MobilityHelper ueMobility;
  ueMobility.Install (ueNodes.Get(0));
  ueMobility.Install (ueNodes.Get(7));
  ueMobility.Install (ueNodes.Get(10));
  ueMobility.Install (ueNodes.Get(17));
  ueMobility.Install (ueNodes.Get(28));
  ueMobility.Install (ueNodes.Get(29));
  ueMobility.Install (ueNodes.Get(38));
  ueMobility.Install (ueNodes.Get(39));
  Ptr<ConstantPositionMobilityModel> m0 = DynamicCast<ConstantPositionMobilityModel>(ueNodes.Get(0)->GetObject<MobilityModel> ());	  
  Ptr<ConstantPositionMobilityModel> m1 = DynamicCast<ConstantPositionMobilityModel>(ueNodes.Get(7)->GetObject<MobilityModel> ());	
  Ptr<ConstantPositionMobilityModel> m2 = DynamicCast<ConstantPositionMobilityModel>(ueNodes.Get(10)->GetObject<MobilityModel> ());	
  Ptr<ConstantPositionMobilityModel> m3 = DynamicCast<ConstantPositionMobilityModel>(ueNodes.Get(17)->GetObject<MobilityModel> ());	
  Ptr<ConstantPositionMobilityModel> m4 = DynamicCast<ConstantPositionMobilityModel>(ueNodes.Get(28)->GetObject<MobilityModel> ());	
  Ptr<ConstantPositionMobilityModel> m5 = DynamicCast<ConstantPositionMobilityModel>(ueNodes.Get(29)->GetObject<MobilityModel> ());	
  Ptr<ConstantPositionMobilityModel> m6 = DynamicCast<ConstantPositionMobilityModel>(ueNodes.Get(38)->GetObject<MobilityModel> ());	
  Ptr<ConstantPositionMobilityModel> m7 = DynamicCast<ConstantPositionMobilityModel>(ueNodes.Get(39)->GetObject<MobilityModel> ());	

 
m0->SetPosition(Vector(200,0,1.6));
m1->SetPosition(Vector(200,0,1.6));

m2->SetPosition(Vector(0,200,1.6)); 
m3->SetPosition(Vector(0,200,1.6)); 

m4->SetPosition(Vector(400,200,1.6));
m5->SetPosition(Vector(400,200,1.6)); 

m6->SetPosition(Vector(200,400,1.6)); 
m7->SetPosition(Vector(200,400,1.6)); 



BuildingsHelper::Install (ueNodes);
  

  Ptr<MyGymEnv> son_server = CreateObject<MyGymEnv> (steptime, numberOfEnbs, numberOfUes, macroEnbBandwidth, openGymPort);
  Ptr<OpenGymInterface> openGymInterface = CreateObject<OpenGymInterface> (openGymPort);

  son_server->SetOpenGymInterface(openGymInterface);

  // Install LTE Devices in eNB and UEs
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbTxPowerDbm));
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes, son_server);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes, son_server);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIfaces;
  ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

  // Attach all UEs to the first eNodeB
  for (uint16_t i = 0; i < numberOfUes; i++)
    {
      lteHelper->AttachToClosestEnb (ueLteDevs.Get(i), enbLteDevs);
    }

  NS_LOG_LOGIC ("setting up applications");

  // Install and start applications on UEs and remote host
  uint16_t dlPort = 10000;
  uint16_t ulPort = 20000;

  DataRateValue dataRateValue = DataRate ("100Mbps");

  // No use
  uint64_t bitRate = dataRateValue.Get ().GetBitRate ();

  // uint32_t packetSize = 1024; //bytes

  NS_LOG_DEBUG ("bit rate " << bitRate);

  // double interPacketInterval = static_cast<double> (packetSize * 8) / bitRate;

  // Time udpInterval = Seconds (interPacketInterval);
  // No use
  Time udpInterval = Seconds (0.01);

  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));

  for (uint32_t u = 0; u < numberOfUes; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
          ApplicationContainer clientApps;
          ApplicationContainer serverApps;
          Ptr<EpcTft> tft = Create<EpcTft> ();

          if (!disableDl)
            {
              ++dlPort;

              NS_LOG_LOGIC ("installing UDP DL app for UE " << u);
              UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort);
              // dlClientHelper.SetAttribute ("Interval", TimeValue (udpInterval));
              // dlClientHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));
              // dlClientHelper.SetAttribute ("MaxPackets", UintegerValue (100));
              clientApps.Add (dlClientHelper.Install (remoteHost));
              PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                                   InetSocketAddress (Ipv4Address::GetAny (), dlPort));
              serverApps.Add (dlPacketSinkHelper.Install (ue));

              EpcTft::PacketFilter dlpf;
              dlpf.localPortStart = dlPort;
              dlpf.localPortEnd = dlPort;
              tft->Add (dlpf);
            }

          if (!disableUl)
            {
              ++ulPort;

              NS_LOG_LOGIC ("installing UDP UL app for UE " << u);
              UdpClientHelper ulClientHelper (remoteHostAddr, ulPort);
              // ulClientHelper.SetAttribute ("Interval", TimeValue (udpInterval));
              // ulClientHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));
              // ulClientHelper.SetAttribute ("MaxPackets", UintegerValue (100));
              clientApps.Add (ulClientHelper.Install (ue));
              PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory",
                                                   InetSocketAddress (Ipv4Address::GetAny (), ulPort));
              serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

              EpcTft::PacketFilter ulpf;
              ulpf.remotePortStart = ulPort;
              ulpf.remotePortEnd = ulPort;
              tft->Add (ulpf);
            }

          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);
          Time startTime = Seconds (startTimeSeconds->GetValue ());
          serverApps.Start (startTime);
          clientApps.Start (startTime);

        } // end for b
    }

  // Add X2 interface
  lteHelper->AddX2Interface (enbNodes);

  // X2-based Handover
  //lteHelper->HandoverRequest (Seconds (0.100), ueLteDevs.Get (0), enbLteDevs.Get (0), enbLteDevs.Get (1));

  // Uncomment to enable PCAP tracing
  // p2ph.EnablePcapAll("lena-x2-handover-measures");

  lteHelper->EnablePhyTraces ();
  // lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();
  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats (son_server);
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));

  // rlcStats->SetAttribute ("DlRlcOutputFilename", StringValue ("Random_crash_joint_1403.txt"));

  Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
  pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));

  // for (uint32_t it = 0; it != enbNodes.GetN(); ++it) {
  //       Ptr < NetDevice > netDevice = enbLteDevs.Get(it);
  //       Ptr < LteEnbNetDevice > enbNetDevice = netDevice -> GetObject < LteEnbNetDevice > ();
  //       Ptr < LteEnbPhy > enbPhy = enbNetDevice -> GetPhy();
  //       enbPhy -> TraceConnectWithoutContext("DlPhyTransmission", MakeBoundCallback( & MyGymEnv::GetPhyStats, son_server));
  //   }

  // connect custom trace sinks for RRC connection establishment and handover notification
  // Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
  //                   MakeCallback (&NotifyConnectionEstablishedEnb));
  // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
  //                   MakeCallback (&NotifyConnectionEstablishedUe));
  // Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
  //                  MakeCallback (&NotifyHandoverStartEnb));
  // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
  //                  MakeCallback (&NotifyHandoverStartUe));
  // Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
  //                  MakeCallback (&NotifyHandoverEndOkEnb));
  // Config::Connect ("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
  //                 MakeCallback (&NotifyHandoverEndOkUe));


  Simulator::Stop (Seconds (simTime));

  //To Reduce xml file size
  // AnimationInterface anim ("buildings.xml"); 
  // anim.SetMaxPktsPerTraceFile(50000);
  // anim.SetMobilityPollInterval(Seconds(0.3)); //Standard : 0.25(s), UE Moving interval : 0.2(s)
  // anim.EnablePacketMetadata (false); //Not include additional information
  
  Simulator::Run ();

  // GtkConfigStore config;
  // config.ConfigureAttributes ();

  // openGymInterface->NotifySimulationEnd();
  Simulator::Destroy ();
  return 0;

}
