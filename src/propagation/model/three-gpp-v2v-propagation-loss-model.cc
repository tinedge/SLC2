/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 SIGNET Lab, Department of Information Engineering,
 * University of Padova
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
 */

#include "three-gpp-v2v-propagation-loss-model.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/string.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ThreeGppV2vPropagationLossModel");

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppV2vUrbanPropagationLossModel);

TypeId
ThreeGppV2vUrbanPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppV2vUrbanPropagationLossModel")
    .SetParent<ThreeGppPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppV2vUrbanPropagationLossModel> ()
    .AddAttribute ("PercType3Vehicles",
                   "The percentage of vehicles of type 3 (i.e., trucks) in the scenario",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&ThreeGppV2vUrbanPropagationLossModel::m_percType3Vehicles),
                   MakeDoubleChecker<double> (0.0, 100.0))
    // .AddAttribute ("Scenario",
		// 		"The available channel scenarios are 'RMa', 'UMa', 'UMi-StreetCanyon', 'InH-OfficeMixed', 'InH-OfficeOpen', 'InH-ShoppingMall'",
		// 		StringValue ("UMa"),
		// 		MakeStringAccessor (&ThreeGppV2vUrbanPropagationLossModel::m_scenario),
		// 		MakeStringChecker ())
  ;
  return tid;
}

ThreeGppV2vUrbanPropagationLossModel::ThreeGppV2vUrbanPropagationLossModel ()
  : ThreeGppPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
  m_uniformVar = CreateObject<UniformRandomVariable> ();
  m_logNorVar = CreateObject<LogNormalRandomVariable> ();

  // set a default channel condition model
  // TODO the default ccm needs buildings, how to do this?
  // m_channelConditionModel = CreateObject<ThreeGppRmaChannelConditionModel> ();
}

ThreeGppV2vUrbanPropagationLossModel::~ThreeGppV2vUrbanPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppV2vUrbanPropagationLossModel::GetLossLos (double /* distance2D */, double distance3D, double /* hUt */, double /* hBs */) const
{
  NS_LOG_FUNCTION (this);

  // compute the pathloss (see 3GPP TR 37.885, Table 6.2.1-1)
  double loss = 38.77 + 16.7 * log10 (distance3D) + 18.2 * log10 (m_frequency / 1e9);
  


  return loss;
}

double
ThreeGppV2vUrbanPropagationLossModel::GetLossNlosv (double distance2D, double distance3D, double hUt, double hBs) const
{
  //  std::cout<<"*********************"<<std::endl;
  NS_LOG_FUNCTION (this);

  // compute the pathloss (see 3GPP TR 37.885, Table 6.2.1-1)
  double loss = GetLossLos (distance2D, distance3D, hUt, hBs) + GetAdditionalNlosvLoss (distance3D, hUt, hBs);

  return loss;
}

double
ThreeGppV2vUrbanPropagationLossModel::GetAdditionalNlosvLoss (double distance3D, double hUt, double hBs) const
{

  
  NS_LOG_FUNCTION (this);
  // From TR 37.885 v15.2.0
  // When a V2V link is in NLOSv, additional vehicle blockage loss is
  // added as follows:
  // 1. The blocker height is the vehicle height which is randomly selected
  // out of the three vehicle types according to the portion of the vehicle
  // types in the simulated scenario.
  double additionalLoss = 0;
  double blockerHeight = 0;
  double mu_a = 0;
  double sigma_a = 0;
  double randomValue = m_uniformVar->GetValue () * 100.0;
  if (randomValue < m_percType3Vehicles)
    {
      // vehicles of type 3 have height 3 meters
      blockerHeight = 3.0;
    }
  else
    {
      // vehicles of type 1 and 2 have height 1.6 meters
      blockerHeight = 1.6;
    }

  // The additional blockage loss is max {0 dB, a log-normal random variable}
  if (std::min (hUt, hBs) > blockerHeight)
    {
      // Case 1: Minimum antenna height value of TX and RX > Blocker height
      additionalLoss = 0;
    }
  else if (std::max (hUt, hBs) < blockerHeight)
    {
      // Case 2: Maximum antenna height value of TX and RX < Blocker height
      mu_a = 9.0 + std::max (0.0, 15 * log10 (distance3D) - 41.0);
      sigma_a = 4.5;
      m_logNorVar->SetAttribute ("Mu", DoubleValue (log (pow (mu_a, 2) / sqrt (pow (sigma_a, 2) + pow (mu_a, 2)))));
      m_logNorVar->SetAttribute ("Sigma", DoubleValue (sqrt (log (pow (sigma_a, 2) / pow (mu_a, 2) + 1))));
      additionalLoss = std::max (0.0, m_logNorVar->GetValue ());
    }
  else
    {
      // Case 3: Otherwise
      mu_a = 5.0 + std::max (0.0, 15 * log10 (distance3D) - 41.0);
      sigma_a = 4.0;

      m_logNorVar->SetAttribute ("Mu", DoubleValue (log (pow (mu_a,2) / sqrt (pow (sigma_a, 2) + pow (mu_a, 2)))));
      m_logNorVar->SetAttribute ("Sigma", DoubleValue (sqrt (log (pow (sigma_a,2) / pow (mu_a, 2) + 1))));
      additionalLoss = std::max (0.0, m_logNorVar->GetValue ());
    }

  return additionalLoss;
}

double
ThreeGppV2vUrbanPropagationLossModel::GetLossNlos (double  distance2D , double distance3D, double  hUt , double  hBs ) const
{
  NS_LOG_FUNCTION (this);
double minloss =0;

  NS_LOG_FUNCTION (this);
if (distance3D <= 0)
	{
		return minloss ;
	}
	if (distance3D < 3*m_lambda)
	{  
		// NS_LOG_UNCOND ("distance not within the far field region => inaccurate propagation loss value");
	}
  
double lossDb =0;
double freqGHz = m_frequency/1e9;

if (m_scenario == "RMa")
		{
			if(distance2D < 10)
			{
				NS_LOG_WARN ("The 2D distance is smaller than 10 meters, the 3GPP RMa model may not be accurate");
			}

			if (hBs < 10 || hBs > 150 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the RMa scenario need to satisfy the following condition, 10 m <= hBS <= 150 m");
			}

			if (hUt < 1 || hUt > 10 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the RMa scenario need to satisfy the following condition, 1 m <= hUT <= 10 m");
			}
			//default base station antenna height is 35 m
			//hBs = 35;
			//default user antenna height is 1.5 m
			//hUt = 1.5;
			// double W = 20; //average street height
			double h = 5; //average building height

			double dBP = 2*M_PI*hBs*hUt*m_frequency/3e8; //break point distance
			double PL1 = 20*log10(40*M_PI*distance3D*freqGHz/3) + std::min(0.03*pow(h,1.72),10.0)*log10(distance3D) - std::min(0.044*pow(h,1.72),14.77) + 0.002*log10(h)*distance3D;

			if(distance2D <= dBP)
			{
				lossDb = PL1;
				// shadowingStd = 4;

			}
			else
			{
				//PL2
				lossDb = PL1 + 40*log10(distance3D/dBP);
				// shadowingStd= 6;
			}

		}

		
		else if (m_scenario == "UMa")
		{
			if(distance2D < 10)
			{
				NS_LOG_UNCOND ("The 2D distance is smaller than 10 meters, the 3GPP UMa model may not be accurate");
			}

			//default base station value is 25 m
			//hBs = 25;

			if (hUt < 1.5 || hUt > 22.5 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the UMa scenario need to satisfy the following condition, 1.5 m <= hUT <= 22.5 m");
			}
			//For UMa, the effective environment height should be computed follow Table7.4.1-1.
			
			double dBP = 4*(hBs)*(hUt)*m_frequency/3e8;
			if(distance2D <= dBP)
			{
				//PL1
				lossDb = 32.4+20*log10(distance3D)+20*log10(freqGHz);
			}
			else
			{
				//PL2
				lossDb = 32.4+40*log10(distance3D)+20*log10(freqGHz)-10*log10(pow(dBP,2)+pow(hBs-hUt,2));
			}
		}

		else if (m_scenario == "UMi-StreetCanyon")
		{

			if(distance2D < 10)
			{
				//NS_LOG_UNCOND ("The 2D distance is smaller than 10 meters, the 3GPP UMi-StreetCanyon model may not be accurate");
			}

			//default base station value is 10 m
			//hBs = 10;

			if (hUt < 1.5 || hUt > 22.5 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the UMi-StreetCanyon scenario need to satisfy the following condition, 1.5 m <= hUT <= 22.5 m");
			}
			double dBP = 4*(hBs-1)*(hUt-1)*m_frequency/3e8;
			if(distance2D <= dBP)
			{
				//PL1
				lossDb = 32.4+21*log10(distance3D)+20*log10(freqGHz);
			}
			else
			{
				//PL2
				lossDb = 32.4+40*log10(distance3D)+20*log10(freqGHz)-9.5*log10(pow(dBP,2)+pow(hBs-hUt,2));
			}

		}
		else if (m_scenario == "InH-OfficeMixed" || m_scenario == "InH-OfficeOpen")
		{
			if(distance3D < 1 || distance3D > 100)
			{
				NS_LOG_UNCOND ("The pathloss might not be accurate since 3GPP InH-Office model LoS condition is accurate only within 3D distance between 1 m and 100 m");
			}

			lossDb = 32.4+17.3*log10(distance3D)+20*log10(freqGHz);
		}

		else if (m_scenario == "InH-ShoppingMall")
		{
			// shadowingCorDistance = 10; //I use the office correlation distance since shopping mall is not in the table.

			if(distance3D < 1 || distance3D > 150)
			{
				NS_LOG_UNCOND ("The pathloss might not be accurate since 3GPP InH-Shopping mall model only supports 3D distance between 1 m and 150 m");\
			}
			lossDb = 32.4+17.3*log10(distance3D)+20*log10(freqGHz);
			// shadowingStd = 2;
		}
		else
		{
			NS_FATAL_ERROR ("Unknown channel condition");
		}

	
		// if(DynamicCast<LteEnbNetDevice> (b->GetObject<Node> ()->GetDevice(0)) != 0)
	  //       {

		// lossDb= 0;
	
	  //       }
	        
		// if(DynamicCast<LteEnbNetDevice> (b->GetObject<Node> ()->GetDevice(0)) != 0)
	  //       {

		// lossDb= 0;
	
	  //       }

	return std::max (lossDb, m_minLoss);
// if(distance2D < 10)
// 			{
// 				// NS_LOG_UNCOND ("The 2D distance is smaller than 10 meters, the 3GPP UMa model may not be accurate");
// 			}

// 			//default base station value is 25 m
// 			//hBs = 25;

// 			if (hUt < 0 || hUt > 22.5 )
// 			{
// 				NS_FATAL_ERROR ("According to table 7.4.1-1, the UMa scenario need to satisfy the following condition, 1.5 m <= hUT <= 22.5 m");
// 			}
// 			//For UMa, the effective environment height should be computed follow Table7.4.1-1.
			
// 			double dBP = 4*(hBs)*(hUt)*m_frequency/3e8;
//       double freqGHz = m_frequency/1e9;
// 			if(distance2D <= dBP)
// 			{
// 				//PL1
// 				loss = 32.4+20*log10(distance3D)+20*log10(freqGHz);
//         // loss =10;
//         // std::cout<<"######"<<std::endl;
// 			}
// 			else
// 			{
// 				//PL2
// 				loss = 32.4+40*log10(distance3D)+20*log10(freqGHz)-10*log10(pow(dBP,2)+pow(hBs-hUt,2));
//         // loss =20;
//         // std::cout<<"@@@@@@"<<std::endl;
//         //  std::cout<<"loss"<<loss<<std::endl;
// 			}

//       loss = std::max (loss, minloss);
//       // std::cout<<"loss"<<loss<<std::endl;
  // return std::max (loss, m_minLoss);
}

double
ThreeGppV2vUrbanPropagationLossModel::GetShadowingStd (Ptr<MobilityModel> /* a */, Ptr<MobilityModel> /* b */, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  double shadowingStd;

  if (cond == ChannelCondition::LosConditionValue::LOS || cond == ChannelCondition::LosConditionValue::NLOSv)
    {
      shadowingStd = 3.0;
      // shadowingStd = 0;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = 4.0;
      // shadowingStd = 0;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return shadowingStd;
}

double
ThreeGppV2vUrbanPropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  double correlationDistance;

  // See 3GPP TR 37.885, Table 6.2.3-1
  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      // correlationDistance = 10;
      correlationDistance = 10;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOSv || cond == ChannelCondition::LosConditionValue::NLOS)
    {
      // correlationDistance = 13;
      correlationDistance = 13;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeGppV2vHighwayPropagationLossModel);

TypeId
ThreeGppV2vHighwayPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeGppV2vHighwayPropagationLossModel")
    .SetParent<ThreeGppV2vUrbanPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeGppV2vHighwayPropagationLossModel> ()
    .AddAttribute ("Scenario",
				"The available channel scenarios are 'RMa', 'UMa', 'UMi-StreetCanyon', 'InH-OfficeMixed', 'InH-OfficeOpen', 'InH-ShoppingMall'",
				StringValue ("UMa"),
				MakeStringAccessor (&ThreeGppV2vUrbanPropagationLossModel::m_scenario),
				MakeStringChecker ())
  ;
  return tid;
}

ThreeGppV2vHighwayPropagationLossModel::ThreeGppV2vHighwayPropagationLossModel ()
  : ThreeGppV2vUrbanPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

ThreeGppV2vHighwayPropagationLossModel::~ThreeGppV2vHighwayPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

double
ThreeGppV2vHighwayPropagationLossModel::GetLossLos (double  distance2D , double distance3D, double  hUt , double  hBs ) const //0504 important
{
  NS_LOG_FUNCTION (this);
double minloss =0;

  NS_LOG_FUNCTION (this);
if (distance3D <= 0)
	{
		return minloss ;
	}
	if (distance3D < 3*m_lambda)
	{  
		// NS_LOG_UNCOND ("distance not within the far field region => inaccurate propagation loss value");
	}
  
double lossDb =0;
double freqGHz = m_frequency/1e9;

if (m_scenario == "RMa")
		{
			if(distance2D < 10)
			{
				NS_LOG_WARN ("The 2D distance is smaller than 10 meters, the 3GPP RMa model may not be accurate");
			}

			if (hBs < 10 || hBs > 150 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the RMa scenario need to satisfy the following condition, 10 m <= hBS <= 150 m");
			}

			if (hUt < 1 || hUt > 10 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the RMa scenario need to satisfy the following condition, 1 m <= hUT <= 10 m");
			}
			//default base station antenna height is 35 m
			//hBs = 35;
			//default user antenna height is 1.5 m
			//hUt = 1.5;
			// double W = 20; //average street height
			double h = 5; //average building height

			double dBP = 2*M_PI*hBs*hUt*m_frequency/3e8; //break point distance
			double PL1 = 20*log10(40*M_PI*distance3D*freqGHz/3) + std::min(0.03*pow(h,1.72),10.0)*log10(distance3D) - std::min(0.044*pow(h,1.72),14.77) + 0.002*log10(h)*distance3D;

			if(distance2D <= dBP)
			{
				lossDb = PL1;
				// shadowingStd = 4;

			}
			else
			{
				//PL2
				lossDb = PL1 + 40*log10(distance3D/dBP);
				// shadowingStd= 6;
			}

		}

		
		else if (m_scenario == "UMa")
		{
			if(distance2D < 10)
			{
				// NS_LOG_UNCOND ("The 2D distance is smaller than 10 meters, the 3GPP UMa model may not be accurate");
			}

			//default base station value is 25 m
			//hBs = 25;

			if (hUt < 1.5 || hUt > 22.5 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the UMa scenario need to satisfy the following condition, 1.5 m <= hUT <= 22.5 m"); //0530
			}
			//For UMa, the effective environment height should be computed follow Table7.4.1-1.
			
			double dBP = 4*(hBs)*(hUt)*m_frequency/3e8;
			if(distance2D <= dBP)
			{
				//PL1
				lossDb = 32.4+20*log10(distance3D)+20*log10(freqGHz);
			}
			else
			{
				//PL2
				lossDb = 32.4+40*log10(distance3D)+20*log10(freqGHz)-10*log10(pow(dBP,2)+pow(hBs-hUt,2));
			}
		}

		else if (m_scenario == "UMi-StreetCanyon")
		{

			if(distance2D < 10)
			{
				//NS_LOG_UNCOND ("The 2D distance is smaller than 10 meters, the 3GPP UMi-StreetCanyon model may not be accurate");
			}

			//default base station value is 10 m
			//hBs = 10;

			if (hUt < 1.5 || hUt > 22.5 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the UMi-StreetCanyon scenario need to satisfy the following condition, 1.5 m <= hUT <= 22.5 m");
			}
			double dBP = 4*(hBs-1)*(hUt-1)*m_frequency/3e8;
			if(distance2D <= dBP)
			{
				//PL1
				lossDb = 32.4+21*log10(distance3D)+20*log10(freqGHz);
			}
			else
			{
				//PL2
				lossDb = 32.4+40*log10(distance3D)+20*log10(freqGHz)-9.5*log10(pow(dBP,2)+pow(hBs-hUt,2));
			}

		}
		else if (m_scenario == "InH-OfficeMixed" || m_scenario == "InH-OfficeOpen")
		{
			if(distance3D < 1 || distance3D > 100)
			{
				NS_LOG_UNCOND ("The pathloss might not be accurate since 3GPP InH-Office model LoS condition is accurate only within 3D distance between 1 m and 100 m");
			}

			lossDb = 32.4+17.3*log10(distance3D)+20*log10(freqGHz);
		}

		else if (m_scenario == "InH-ShoppingMall")
		{
			// shadowingCorDistance = 10; //I use the office correlation distance since shopping mall is not in the table.

			if(distance3D < 1 || distance3D > 150)
			{
				NS_LOG_UNCOND ("The pathloss might not be accurate since 3GPP InH-Shopping mall model only supports 3D distance between 1 m and 150 m");\
			}
			lossDb = 32.4+17.3*log10(distance3D)+20*log10(freqGHz);
			// shadowingStd = 2;
		}
		else
		{
			NS_FATAL_ERROR ("Unknown channel condition");
		}

	
		// if(DynamicCast<LteEnbNetDevice> (b->GetObject<Node> ()->GetDevice(0)) != 0)
	  //       {

		// lossDb= 0;
	
	  //       }
	        
		// if(DynamicCast<LteEnbNetDevice> (b->GetObject<Node> ()->GetDevice(0)) != 0)
	  //       {

		// lossDb= 0;
	
	  //       }

	return std::max (lossDb, m_minLoss);
// if(distance2D < 10)
// 			{
// 				// NS_LOG_UNCOND ("The 2D distance is smaller than 10 meters, the 3GPP UMa model may not be accurate");
// 			}

// 			//default base station value is 25 m
// 			//hBs = 25;

// 			if (hUt < 0 || hUt > 22.5 )
// 			{
// 				NS_FATAL_ERROR ("According to table 7.4.1-1, the UMa scenario need to satisfy the following condition, 1.5 m <= hUT <= 22.5 m");
// 			}
// 			//For UMa, the effective environment height should be computed follow Table7.4.1-1.
			
// 			double dBP = 4*(hBs)*(hUt)*m_frequency/3e8;
//       double freqGHz = m_frequency/1e9;
// 			if(distance2D <= dBP)
// 			{
// 				//PL1
// 				loss = 32.4+20*log10(distance3D)+20*log10(freqGHz);
//         // loss =10;
//         // std::cout<<"######"<<std::endl;
// 			}
// 			else
// 			{
// 				//PL2
// 				loss = 32.4+40*log10(distance3D)+20*log10(freqGHz)-10*log10(pow(dBP,2)+pow(hBs-hUt,2));
//         // loss =20;
//         // std::cout<<"@@@@@@"<<std::endl;
//         //  std::cout<<"loss"<<loss<<std::endl;
// 			}

//       loss = std::max (loss, minloss);
//       // std::cout<<"loss"<<loss<<std::endl;
  // return std::max (loss, m_minLoss);
}


double
ThreeGppV2vHighwayPropagationLossModel::GetLossNlos (double  distance2D , double distance3D, double  hUt , double  hBs ) const
{
  NS_LOG_FUNCTION (this);
double minloss =0;

  NS_LOG_FUNCTION (this);
if (distance3D <= 0)
	{
		return minloss ;
	}
	if (distance3D < 3*m_lambda)
	{  
		// NS_LOG_UNCOND ("distance not within the far field region => inaccurate propagation loss value");
	}
  
double lossDb =0;
double freqGHz = m_frequency/1e9;

if (m_scenario == "RMa")
		{
			if(distance2D < 10)
			{
				NS_LOG_WARN ("The 2D distance is smaller than 10 meters, the 3GPP RMa model may not be accurate");
			}

			if (hBs < 10 || hBs > 150 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the RMa scenario need to satisfy the following condition, 10 m <= hBS <= 150 m");
			}

			if (hUt < 1 || hUt > 10 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the RMa scenario need to satisfy the following condition, 1 m <= hUT <= 10 m");
			}
			//default base station antenna height is 35 m
			//hBs = 35;
			//default user antenna height is 1.5 m
			//hUt = 1.5;
			// double W = 20; //average street height
			double h = 5; //average building height

			double dBP = 2*M_PI*hBs*hUt*m_frequency/3e8; //break point distance
			double PL1 = 20*log10(40*M_PI*distance3D*freqGHz/3) + std::min(0.03*pow(h,1.72),10.0)*log10(distance3D) - std::min(0.044*pow(h,1.72),14.77) + 0.002*log10(h)*distance3D;

			if(distance2D <= dBP)
			{
				lossDb = PL1;
				// shadowingStd = 4;

			}
			else
			{
				//PL2
				lossDb = PL1 + 40*log10(distance3D/dBP);
				// shadowingStd= 6;
			}

		}

		
		else if (m_scenario == "UMa")
		{
			if(distance2D < 10)
			{
				// NS_LOG_UNCOND ("The 2D distance is smaller than 10 meters, the 3GPP UMa model may not be accurate");
			}

			//default base station value is 25 m
			//hBs = 25;

			if (hUt < 1.5 || hUt > 22.5 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the UMa scenario need to satisfy the following condition, 1.5 m <= hUT <= 22.5 m");
			}
			//For UMa, the effective environment height should be computed follow Table7.4.1-1.
			
			double dBP = 4*(hBs)*(hUt)*m_frequency/3e8;
			if(distance2D <= dBP)
			{
				//PL1
				lossDb = 65+20*log10(distance3D)+20*log10(freqGHz);
				// lossDb = 65+20*log10(distance3D)+20*log10(freqGHz);

			}
			else
			{
				//PL2
				// lossDb = 65+40*log10(distance3D)+20*log10(freqGHz)-10*log10(pow(dBP,2)+pow(hBs-hUt,2));
				// lossDb = 65+20*log10(distance3D)+20*log10(freqGHz);
				lossDb = 65+40*log10(distance3D)+20*log10(freqGHz)-10*log10(pow(dBP,2)+pow(hBs-hUt,2));
				
			}
		}

		else if (m_scenario == "UMi-StreetCanyon")
		{

			if(distance2D < 10)
			{
				//NS_LOG_UNCOND ("The 2D distance is smaller than 10 meters, the 3GPP UMi-StreetCanyon model may not be accurate");
			}

			//default base station value is 10 m
			//hBs = 10;

			if (hUt < 1.5 || hUt > 22.5 )
			{
				NS_FATAL_ERROR ("According to table 7.4.1-1, the UMi-StreetCanyon scenario need to satisfy the following condition, 1.5 m <= hUT <= 22.5 m");
			}
			double dBP = 4*(hBs-1)*(hUt-1)*m_frequency/3e8;
			if(distance2D <= dBP)
			{
				//PL1
				lossDb = 32.4+21*log10(distance3D)+20*log10(freqGHz);
			}
			else
			{
				//PL2
				lossDb = 32.4+40*log10(distance3D)+20*log10(freqGHz)-9.5*log10(pow(dBP,2)+pow(hBs-hUt,2));
			}

		}
		else if (m_scenario == "InH-OfficeMixed" || m_scenario == "InH-OfficeOpen")
		{
			if(distance3D < 1 || distance3D > 100)
			{
				NS_LOG_UNCOND ("The pathloss might not be accurate since 3GPP InH-Office model LoS condition is accurate only within 3D distance between 1 m and 100 m");
			}

			lossDb = 32.4+17.3*log10(distance3D)+20*log10(freqGHz);
		}

		else if (m_scenario == "InH-ShoppingMall")
		{
			// shadowingCorDistance = 10; //I use the office correlation distance since shopping mall is not in the table.

			if(distance3D < 1 || distance3D > 150)
			{
				NS_LOG_UNCOND ("The pathloss might not be accurate since 3GPP InH-Shopping mall model only supports 3D distance between 1 m and 150 m");\
			}
			lossDb = 32.4+17.3*log10(distance3D)+20*log10(freqGHz);
			// shadowingStd = 2;
		}
		else
		{
			NS_FATAL_ERROR ("Unknown channel condition");
		}

	
		// if(DynamicCast<LteEnbNetDevice> (b->GetObject<Node> ()->GetDevice(0)) != 0)
	  //       {

		// lossDb= 0;
	
	  //       }
	        
		// if(DynamicCast<LteEnbNetDevice> (b->GetObject<Node> ()->GetDevice(0)) != 0)
	  //       {

		// lossDb= 0;
	
	  //       }

	return std::max (lossDb, m_minLoss);
// if(distance2D < 10)
// 			{
// 				// NS_LOG_UNCOND ("The 2D distance is smaller than 10 meters, the 3GPP UMa model may not be accurate");
// 			}

// 			//default base station value is 25 m
// 			//hBs = 25;

// 			if (hUt < 0 || hUt > 22.5 )
// 			{
// 				NS_FATAL_ERROR ("According to table 7.4.1-1, the UMa scenario need to satisfy the following condition, 1.5 m <= hUT <= 22.5 m");
// 			}
// 			//For UMa, the effective environment height should be computed follow Table7.4.1-1.
			
// 			double dBP = 4*(hBs)*(hUt)*m_frequency/3e8;
//       double freqGHz = m_frequency/1e9;
// 			if(distance2D <= dBP)
// 			{
// 				//PL1
// 				loss = 32.4+20*log10(distance3D)+20*log10(freqGHz);
//         // loss =10;
//         // std::cout<<"######"<<std::endl;
// 			}
// 			else
// 			{
// 				//PL2
// 				loss = 32.4+40*log10(distance3D)+20*log10(freqGHz)-10*log10(pow(dBP,2)+pow(hBs-hUt,2));
//         // loss =20;
//         // std::cout<<"@@@@@@"<<std::endl;
//         //  std::cout<<"loss"<<loss<<std::endl;
// 			}

//       loss = std::max (loss, minloss);
//       // std::cout<<"loss"<<loss<<std::endl;
  // return std::max (loss, m_minLoss);
}


} // namespace ns3
