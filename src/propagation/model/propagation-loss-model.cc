/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Contributions: Timo Bingmann <timo.bingmann@student.kit.edu>
 * Contributions: Tom Hewer <tomhewer@mac.com> for Two Ray Ground Model
 *                Pavel Boyko <boyko@iitp.ru> for matrix
 */

#include "propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include <cmath>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <ns3/object.h>
#include <ns3/double.h>
#include <ns3/antenna-model.h>
#include <ns3/cosine-antenna-model.h>
#include <ns3/angles.h>
#include <ns3/core-module.h>
#include <ns3/lte-enb-net-device.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("PropagationLossModel");

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (PropagationLossModel);

TypeId 
PropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PropagationLossModel")
    .SetParent<Object> ()
    .SetGroupName ("Propagation")
  ;
  return tid;
}

PropagationLossModel::PropagationLossModel ()
  : m_next (0)
{
}

PropagationLossModel::~PropagationLossModel ()
{
}

void
PropagationLossModel::SetNext (Ptr<PropagationLossModel> next)
{
  m_next = next;
}

Ptr<PropagationLossModel>
PropagationLossModel::GetNext ()
{
  return m_next;
}

double
PropagationLossModel::CalcRxPower (double txPowerDbm,
                                   Ptr<MobilityModel> a,
                                   Ptr<MobilityModel> b) const
{
  double self = DoCalcRxPower (txPowerDbm, a, b);
  if (m_next != 0)
    {
      self = m_next->CalcRxPower (self, a, b);
    }
  return self;
}

int64_t
PropagationLossModel::AssignStreams (int64_t stream)
{
  int64_t currentStream = stream;
  currentStream += DoAssignStreams (stream);
  if (m_next != 0)
    {
      currentStream += m_next->AssignStreams (currentStream);
    }
  return (currentStream - stream);
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (RandomPropagationLossModel);

TypeId 
RandomPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RandomPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<RandomPropagationLossModel> ()
    .AddAttribute ("Variable", "The random variable used to pick a loss every time CalcRxPower is invoked.",
                   StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
                   MakePointerAccessor (&RandomPropagationLossModel::m_variable),
                   MakePointerChecker<RandomVariableStream> ())
  ;
  return tid;
}
RandomPropagationLossModel::RandomPropagationLossModel ()
  : PropagationLossModel ()
{
}

RandomPropagationLossModel::~RandomPropagationLossModel ()
{
}

double
RandomPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                           Ptr<MobilityModel> a,
                                           Ptr<MobilityModel> b) const
{
  double rxc = -m_variable->GetValue ();
  NS_LOG_DEBUG ("attenuation coefficient="<<rxc<<"Db");
  return txPowerDbm + rxc;
}

int64_t
RandomPropagationLossModel::DoAssignStreams (int64_t stream)
{
  m_variable->SetStream (stream);
  return 1;
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (FriisPropagationLossModel);

TypeId 
FriisPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::FriisPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<FriisPropagationLossModel> ()
    .AddAttribute ("Frequency", 
                   "The carrier frequency (in Hz) at which propagation occurs  (default is 5.15 GHz).",
                   DoubleValue (5.150e9),
                   MakeDoubleAccessor (&FriisPropagationLossModel::SetFrequency,
                                       &FriisPropagationLossModel::GetFrequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SystemLoss", "The system loss",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&FriisPropagationLossModel::m_systemLoss),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MinLoss", 
                   "The minimum value (dB) of the total loss, used at short ranges. Note: ",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&FriisPropagationLossModel::SetMinLoss,
                                       &FriisPropagationLossModel::GetMinLoss),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

FriisPropagationLossModel::FriisPropagationLossModel ()
{
}
void
FriisPropagationLossModel::SetSystemLoss (double systemLoss)
{
  m_systemLoss = systemLoss;
}
double
FriisPropagationLossModel::GetSystemLoss (void) const
{
  return m_systemLoss;
}
void
FriisPropagationLossModel::SetMinLoss (double minLoss)
{
  m_minLoss = minLoss;
}
double
FriisPropagationLossModel::GetMinLoss (void) const
{
  return m_minLoss;
}

void
FriisPropagationLossModel::SetFrequency (double frequency)
{
  m_frequency = frequency;
  static const double C = 299792458.0; // speed of light in vacuum
  m_lambda = C / frequency;
}

double
FriisPropagationLossModel::GetFrequency (void) const
{
  return m_frequency;
}

double
FriisPropagationLossModel::DbmToW (double dbm) const
{
  double mw = std::pow (10.0,dbm/10.0);
  return mw / 1000.0;
}

double
FriisPropagationLossModel::DbmFromW (double w) const
{
  double dbm = std::log10 (w * 1000.0) * 10.0;
  return dbm;
}

double 
FriisPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                          Ptr<MobilityModel> a,
                                          Ptr<MobilityModel> b) const
{
  /*
   * Friis free space equation:
   * where Pt, Gr, Gr and P are in Watt units
   * L is in meter units.
   *
   *    P     Gt * Gr * (lambda^2)
   *   --- = ---------------------
   *    Pt     (4 * pi * d)^2 * L
   *
   * Gt: tx gain (unit-less)
   * Gr: rx gain (unit-less)
   * Pt: tx power (W)
   * d: distance (m)
   * L: system loss
   * lambda: wavelength (m)
   *
   * Here, we ignore tx and rx gain and the input and output values 
   * are in dB or dBm:
   *
   *                           lambda^2
   * rx = tx +  10 log10 (-------------------)
   *                       (4 * pi * d)^2 * L
   *
   * rx: rx power (dB)
   * tx: tx power (dB)
   * d: distance (m)
   * L: system loss (unit-less)
   * lambda: wavelength (m)
   */
  double distance = a->GetDistanceFrom (b);
  if (distance < 3*m_lambda)
    {
      NS_LOG_WARN ("distance not within the far field region => inaccurate propagation loss value");
    }
  if (distance <= 0)
    {
      return txPowerDbm - m_minLoss;
    }
  double numerator = m_lambda * m_lambda;
  double denominator = 16 * M_PI * M_PI * distance * distance * m_systemLoss;
  double lossDb = -10 * log10 (numerator / denominator);
  NS_LOG_DEBUG ("distance=" << distance<< "m, loss=" << lossDb <<"dB");
  return txPowerDbm - std::max (lossDb, m_minLoss);
}

int64_t
FriisPropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}

// ------------------------------------------------------------------------- //
// -- Two-Ray Ground Model ported from NS-2 -- tomhewer@mac.com -- Nov09 //

NS_OBJECT_ENSURE_REGISTERED (TwoRayGroundPropagationLossModel);

TypeId 
TwoRayGroundPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TwoRayGroundPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<TwoRayGroundPropagationLossModel> ()
    .AddAttribute ("Frequency", 
                   "The carrier frequency (in Hz) at which propagation occurs  (default is 5.15 GHz).",
                   DoubleValue (5.150e9),
                   MakeDoubleAccessor (&TwoRayGroundPropagationLossModel::SetFrequency,
                                       &TwoRayGroundPropagationLossModel::GetFrequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SystemLoss", "The system loss",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&TwoRayGroundPropagationLossModel::m_systemLoss),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MinDistance",
                   "The distance under which the propagation model refuses to give results (m)",
                   DoubleValue (0.5),
                   MakeDoubleAccessor (&TwoRayGroundPropagationLossModel::SetMinDistance,
                                       &TwoRayGroundPropagationLossModel::GetMinDistance),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("HeightAboveZ",
                   "The height of the antenna (m) above the node's Z coordinate",
                   DoubleValue (0),
                   MakeDoubleAccessor (&TwoRayGroundPropagationLossModel::m_heightAboveZ),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

TwoRayGroundPropagationLossModel::TwoRayGroundPropagationLossModel ()
{
}
void
TwoRayGroundPropagationLossModel::SetSystemLoss (double systemLoss)
{
  m_systemLoss = systemLoss;
}
double
TwoRayGroundPropagationLossModel::GetSystemLoss (void) const
{
  return m_systemLoss;
}
void
TwoRayGroundPropagationLossModel::SetMinDistance (double minDistance)
{
  m_minDistance = minDistance;
}
double
TwoRayGroundPropagationLossModel::GetMinDistance (void) const
{
  return m_minDistance;
}
void
TwoRayGroundPropagationLossModel::SetHeightAboveZ (double heightAboveZ)
{
  m_heightAboveZ = heightAboveZ;
}

void
TwoRayGroundPropagationLossModel::SetFrequency (double frequency)
{
  m_frequency = frequency;
  static const double C = 299792458.0; // speed of light in vacuum
  m_lambda = C / frequency;
}

double
TwoRayGroundPropagationLossModel::GetFrequency (void) const
{
  return m_frequency;
}

double 
TwoRayGroundPropagationLossModel::DbmToW (double dbm) const
{
  double mw = std::pow (10.0,dbm / 10.0);
  return mw / 1000.0;
}

double
TwoRayGroundPropagationLossModel::DbmFromW (double w) const
{
  double dbm = std::log10 (w * 1000.0) * 10.0;
  return dbm;
}

double 
TwoRayGroundPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                                 Ptr<MobilityModel> a,
                                                 Ptr<MobilityModel> b) const
{
  /*
   * Two-Ray Ground equation:
   *
   * where Pt, Gt and Gr are in dBm units
   * L, Ht and Hr are in meter units.
   *
   *   Pr      Gt * Gr * (Ht^2 * Hr^2)
   *   -- =  (-------------------------)
   *   Pt            d^4 * L
   *
   * Gt: tx gain (unit-less)
   * Gr: rx gain (unit-less)
   * Pt: tx power (dBm)
   * d: distance (m)
   * L: system loss
   * Ht: Tx antenna height (m)
   * Hr: Rx antenna height (m)
   * lambda: wavelength (m)
   *
   * As with the Friis model we ignore tx and rx gain and output values
   * are in dB or dBm
   *
   *                      (Ht * Ht) * (Hr * Hr)
   * rx = tx + 10 log10 (-----------------------)
   *                      (d * d * d * d) * L
   */
  double distance = a->GetDistanceFrom (b);
  if (distance <= m_minDistance)
    {
      return txPowerDbm;
    }

  // Set the height of the Tx and Rx antennae
  double txAntHeight = a->GetPosition ().z + m_heightAboveZ;
  double rxAntHeight = b->GetPosition ().z + m_heightAboveZ;

  // Calculate a crossover distance, under which we use Friis
  /*
   * 
   * dCross = (4 * pi * Ht * Hr) / lambda
   *
   */

  double dCross = (4 * M_PI * txAntHeight * rxAntHeight) / m_lambda;
  double tmp = 0;
  if (distance <= dCross)
    {
      // We use Friis
      double numerator = m_lambda * m_lambda;
      tmp = M_PI * distance;
      double denominator = 16 * tmp * tmp * m_systemLoss;
      double pr = 10 * std::log10 (numerator / denominator);
      NS_LOG_DEBUG ("Receiver within crossover (" << dCross << "m) for Two_ray path; using Friis");
      NS_LOG_DEBUG ("distance=" << distance << "m, attenuation coefficient=" << pr << "dB");
      return txPowerDbm + pr;
    }
  else   // Use Two-Ray Pathloss
    {
      tmp = txAntHeight * rxAntHeight;
      double rayNumerator = tmp * tmp;
      tmp = distance * distance;
      double rayDenominator = tmp * tmp * m_systemLoss;
      double rayPr = 10 * std::log10 (rayNumerator / rayDenominator);
      NS_LOG_DEBUG ("distance=" << distance << "m, attenuation coefficient=" << rayPr << "dB");
      return txPowerDbm + rayPr;

    }
}

int64_t
TwoRayGroundPropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}


// ------------------------------------------------------------------------- //
// -- Mmwave propagation loss model -- //

NS_OBJECT_ENSURE_REGISTERED (MmWavePropagationLossModel);

TypeId 
MmWavePropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MmWavePropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<MmWavePropagationLossModel> ()
    .AddAttribute ("Frequency", 
                   "The carrier frequency (in Hz) at which propagation occurs  (default is 300 GHz).",
                   DoubleValue (3.000e10),
                   MakeDoubleAccessor (&MmWavePropagationLossModel::SetFrequency,
                                       &MmWavePropagationLossModel::GetFrequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Scenario",
				"The available channel scenarios are 'RMa', 'UMa', 'UMi-StreetCanyon', 'InH-OfficeMixed', 'InH-OfficeOpen', 'InH-ShoppingMall'",
				StringValue ("UMa"),
				MakeStringAccessor (&MmWavePropagationLossModel::m_scenario),
				MakeStringChecker ())
    .AddAttribute ("MinLoss",
                   "The minimum value (dB) of the total loss, used at short ranges.",
                   DoubleValue (0.0),
                   MakeDoubleAccessor (&MmWavePropagationLossModel::SetMinLoss,
                                       &MmWavePropagationLossModel::GetMinLoss),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

MmWavePropagationLossModel::MmWavePropagationLossModel ()
{
}


void
MmWavePropagationLossModel::SetFrequency (double frequency)
{
  m_frequency = frequency;
  static const double C = 299792458.0; // speed of light in vacuum
  m_lambda = C / frequency;
}

void
MmWavePropagationLossModel::SetMinLoss (double minLoss)
{
  m_minLoss = minLoss;
}
double
MmWavePropagationLossModel::GetMinLoss (void) const
{
  return m_minLoss;
}


double
MmWavePropagationLossModel::GetFrequency (void) const
{
  return m_frequency;
}

double 
MmWavePropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                                 Ptr<MobilityModel> a,
                                                 Ptr<MobilityModel> b) const
{
  return (txPowerDbm - GetLoss (a, b));
}

// LTE 환경에 맞게 변형
/////////////////////////
std::tuple< Ptr<MobilityModel>, Ptr<MobilityModel> >
MmWavePropagationLossModel::GetEnbUePair(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{
	Ptr<MobilityModel> ueMob=0, enbMob=0;

	// if(DynamicCast<LteEnbNetDevice> (a->GetObject<LteEnbNetDevice> ()) != 0)
	// {
	// 	// for sure it is downlink
	// 	enbMob = a;
	// 	ueMob = b;
	// }
	// else if(DynamicCast<LteEnbNetDevice> (b->GetObject<LteEnbNetDevice> ()) != 0)
	// {
	// 	// for sure it is uplink
	// 	ueMob = a;
	// 	enbMob = b;
	// }

  // ueMob = a;
	// enbMob = b;

  // Down Link
  ueMob = b;
	enbMob = a;

	return std::make_tuple(enbMob, ueMob);
}
/////////////////////////


// LTE 환경에 맞게 변형
/////////////////////////
double
MmWavePropagationLossModel::GetLoss (Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
{

	double distance3D = a->GetDistanceFrom (b);

	if (distance3D <= 0)
	{
		return  m_minLoss;
	}
	if (distance3D < 3*m_lambda)
	{
		NS_LOG_UNCOND ("distance not within the far field region => inaccurate propagation loss value");
	}

	Ptr<MobilityModel> ueMob, enbMob;
	
	auto returnParams = GetEnbUePair(a, b);

	enbMob = std::get<0>(returnParams);
	ueMob =  std::get<1>(returnParams);

  //오류 없애는 용도
  if(ueMob->GetPosition().z > 10){
    Ptr<MobilityModel> temp;
    temp = ueMob;
    ueMob = enbMob;
    enbMob = temp;
  }

	Vector uePos = ueMob->GetPosition();
	Vector enbPos = enbMob->GetPosition();
	double x = uePos.x-enbPos.x;
	double y = uePos.y-enbPos.y;
	double distance2D = sqrt (x*x +y*y);
	double hBs = enbPos.z;
	double hUt = uePos.z;


  // std::cout<<"hBs: "<<hBs<<"  hUt: "<<hUt<<std::endl;
  ///////////////

		double lossDb = 0;
		double freqGHz = m_frequency/1e9;

		// double shadowingStd = 0;
		// double shadowingCorDistance = 0;
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
}
/////////////////////////





int64_t
MmWavePropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}


// ------------------------------------------------------------------------- //



// ------------------------------------------------------------------------- //
// -- Terahertz propagation loss model -- //

NS_OBJECT_ENSURE_REGISTERED (TerahertzPropagationLossModel);

TypeId 
TerahertzPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TerahertzPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<TerahertzPropagationLossModel> ()
    .AddAttribute ("Frequency", 
                   "The carrier frequency (in Hz) at which propagation occurs  (default is 300 GHz).",
                   DoubleValue (3.000e11),
                   MakeDoubleAccessor (&TerahertzPropagationLossModel::SetFrequency,
                                       &TerahertzPropagationLossModel::GetFrequency),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

TerahertzPropagationLossModel::TerahertzPropagationLossModel ()
{
}


void
TerahertzPropagationLossModel::SetFrequency (double frequency)
{
  m_frequency = frequency;
  static const double C = 299792458.0; // speed of light in vacuum
  m_lambda = C / frequency;
}


double
TerahertzPropagationLossModel::GetFrequency (void) const
{
  return m_frequency;
}


double 
TerahertzPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                                 Ptr<MobilityModel> a,
                                                 Ptr<MobilityModel> b) const
{

  NS_ASSERT (a);
  NS_ASSERT (b);
  double d = a->GetDistanceFrom (b);
  double frequency = TerahertzPropagationLossModel::GetFrequency();
  double lossDb = 10 * std::log10 (CalculateSpreadLoss (frequency, d)) + 10 * std::log10 (CalculateAbsLoss (frequency, d));

  return txPowerDbm - lossDb;

}

double
TerahertzPropagationLossModel::CalculateSpreadLoss (double f, double d) const
{
  NS_ASSERT (d >= 0);
 f = 3.000e11;
  if (d == 0)
    {
      return 0;
    }

  NS_ASSERT (f > 0);
  double loss_sqrt = (4 * M_PI * f * d) / 3e8;
  double loss = loss_sqrt * loss_sqrt;
  return loss;
}

double
TerahertzPropagationLossModel::CalculateAbsLoss (double f, double d) const
{       f = 3.000e11;
  std::ifstream AbsCoefile;
  AbsCoefile.open ("src/propagation/model/data_AbsCoe.txt", std::ifstream::in);
  if (!AbsCoefile.is_open ())
    {
      NS_FATAL_ERROR ("TerahertzPropagationLossModel::CalculateAbsLoss: open data_AbsCoe.txt failed 1");
    }

  std::ifstream frequencyfile;
  frequencyfile.open ("src/propagation/model/data_frequency.txt", std::ifstream::in);
  if (!frequencyfile.is_open ())
    {
      NS_FATAL_ERROR ("TerahertzPropagationLossModel::CalculateAbsLoss: open data_frequency.txt failed");
    }
  double f_ite;
  double k_ite;
  double kf = 0.0;
  double loss = 0.0;
  int i = 0;
  int j = 0;


  while (frequencyfile >> f_ite)
    {
      if (f_ite < f - 9.894e8 || f_ite > f + 9.894e8)
        {
          j++;
        }
      else
        {
          break;
        }
    }
  while (AbsCoefile >> k_ite)
    {
      if (i != j)
        {
          i++;
        }
      else
        {
          kf = k_ite;
          break;
        }
      NS_ASSERT (d >= 0);

      if (d == 0)
        {
          return 0;
        }
    }
  NS_ASSERT (f > 0);
  loss = exp (kf * d);
  return loss;
}

int64_t
TerahertzPropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}





// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (LogDistancePropagationLossModel);

TypeId
LogDistancePropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::LogDistancePropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<LogDistancePropagationLossModel> ()
    .AddAttribute ("Exponent",
                   "The exponent of the Path Loss propagation model",
                   DoubleValue (3.0),
                   MakeDoubleAccessor (&LogDistancePropagationLossModel::m_exponent),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ReferenceDistance",
                   "The distance at which the reference loss is calculated (m)",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&LogDistancePropagationLossModel::m_referenceDistance),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ReferenceLoss",
                   "The reference loss at reference distance (dB). (Default is Friis at 1m with 5.15 GHz)",
                   DoubleValue (46.6777),
                   MakeDoubleAccessor (&LogDistancePropagationLossModel::m_referenceLoss),
                   MakeDoubleChecker<double> ())
  ;
  return tid;

}

LogDistancePropagationLossModel::LogDistancePropagationLossModel ()
{
}

void
LogDistancePropagationLossModel::SetPathLossExponent (double n)
{
  m_exponent = n;
}
void
LogDistancePropagationLossModel::SetReference (double referenceDistance, double referenceLoss)
{
  m_referenceDistance = referenceDistance;
  m_referenceLoss = referenceLoss;
}
double
LogDistancePropagationLossModel::GetPathLossExponent (void) const
{
  return m_exponent;
}

double
LogDistancePropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                                Ptr<MobilityModel> a,
                                                Ptr<MobilityModel> b) const
{
  double distance = a->GetDistanceFrom (b);
  if (distance <= m_referenceDistance)
    {
      return txPowerDbm - m_referenceLoss;
    }
  /**
   * The formula is:
   * rx = 10 * log (Pr0(tx)) - n * 10 * log (d/d0)
   *
   * Pr0: rx power at reference distance d0 (W)
   * d0: reference distance: 1.0 (m)
   * d: distance (m)
   * tx: tx power (dB)
   * rx: dB
   *
   * Which, in our case is:
   *
   * rx = rx0(tx) - 10 * n * log (d/d0)
   */
  double pathLossDb = 10 * m_exponent * std::log10 (distance / m_referenceDistance);
  double rxc = -m_referenceLoss - pathLossDb;
  NS_LOG_DEBUG ("distance="<<distance<<"m, reference-attenuation="<< -m_referenceLoss<<"dB, "<<
                "attenuation coefficient="<<rxc<<"db");
  return txPowerDbm + rxc;
}

int64_t
LogDistancePropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (ThreeLogDistancePropagationLossModel);

TypeId
ThreeLogDistancePropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ThreeLogDistancePropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<ThreeLogDistancePropagationLossModel> ()
    .AddAttribute ("Distance0",
                   "Beginning of the first (near) distance field",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&ThreeLogDistancePropagationLossModel::m_distance0),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Distance1",
                   "Beginning of the second (middle) distance field.",
                   DoubleValue (200.0),
                   MakeDoubleAccessor (&ThreeLogDistancePropagationLossModel::m_distance1),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Distance2",
                   "Beginning of the third (far) distance field.",
                   DoubleValue (500.0),
                   MakeDoubleAccessor (&ThreeLogDistancePropagationLossModel::m_distance2),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Exponent0",
                   "The exponent for the first field.",
                   DoubleValue (1.9),
                   MakeDoubleAccessor (&ThreeLogDistancePropagationLossModel::m_exponent0),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Exponent1",
                   "The exponent for the second field.",
                   DoubleValue (3.8),
                   MakeDoubleAccessor (&ThreeLogDistancePropagationLossModel::m_exponent1),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Exponent2",
                   "The exponent for the third field.",
                   DoubleValue (3.8),
                   MakeDoubleAccessor (&ThreeLogDistancePropagationLossModel::m_exponent2),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ReferenceLoss",
                   "The reference loss at distance d0 (dB). (Default is Friis at 1m with 5.15 GHz)",
                   DoubleValue (46.6777),
                   MakeDoubleAccessor (&ThreeLogDistancePropagationLossModel::m_referenceLoss),
                   MakeDoubleChecker<double> ())
  ;
  return tid;

}

ThreeLogDistancePropagationLossModel::ThreeLogDistancePropagationLossModel ()
{
}

double 
ThreeLogDistancePropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                                     Ptr<MobilityModel> a,
                                                     Ptr<MobilityModel> b) const
{
  double distance = a->GetDistanceFrom (b);
  NS_ASSERT (distance >= 0);

  // See doxygen comments for the formula and explanation

  double pathLossDb;

  if (distance < m_distance0)
    {
      pathLossDb = 0;
    }
  else if (distance < m_distance1)
    {
      pathLossDb = m_referenceLoss
        + 10 * m_exponent0 * std::log10 (distance / m_distance0);
    }
  else if (distance < m_distance2)
    {
      pathLossDb = m_referenceLoss
        + 10 * m_exponent0 * std::log10 (m_distance1 / m_distance0)
        + 10 * m_exponent1 * std::log10 (distance / m_distance1);
    }
  else
    {
      pathLossDb = m_referenceLoss
        + 10 * m_exponent0 * std::log10 (m_distance1 / m_distance0)
        + 10 * m_exponent1 * std::log10 (m_distance2 / m_distance1)
        + 10 * m_exponent2 * std::log10 (distance / m_distance2);
    }

  NS_LOG_DEBUG ("ThreeLogDistance distance=" << distance << "m, " <<
                "attenuation=" << pathLossDb << "dB");

  return txPowerDbm - pathLossDb;
}

int64_t
ThreeLogDistancePropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (NakagamiPropagationLossModel);

TypeId
NakagamiPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NakagamiPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<NakagamiPropagationLossModel> ()
    .AddAttribute ("Distance1",
                   "Beginning of the second distance field. Default is 80m.",
                   DoubleValue (80.0),
                   MakeDoubleAccessor (&NakagamiPropagationLossModel::m_distance1),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Distance2",
                   "Beginning of the third distance field. Default is 200m.",
                   DoubleValue (200.0),
                   MakeDoubleAccessor (&NakagamiPropagationLossModel::m_distance2),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("m0",
                   "m0 for distances smaller than Distance1. Default is 1.5.",
                   DoubleValue (1.5),
                   MakeDoubleAccessor (&NakagamiPropagationLossModel::m_m0),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("m1",
                   "m1 for distances smaller than Distance2. Default is 0.75.",
                   DoubleValue (0.75),
                   MakeDoubleAccessor (&NakagamiPropagationLossModel::m_m1),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("m2",
                   "m2 for distances greater than Distance2. Default is 0.75.",
                   DoubleValue (0.75),
                   MakeDoubleAccessor (&NakagamiPropagationLossModel::m_m2),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("ErlangRv",
                   "Access to the underlying ErlangRandomVariable",
                   StringValue ("ns3::ErlangRandomVariable"),
                   MakePointerAccessor (&NakagamiPropagationLossModel::m_erlangRandomVariable),
                   MakePointerChecker<ErlangRandomVariable> ())
    .AddAttribute ("GammaRv",
                   "Access to the underlying GammaRandomVariable",
                   StringValue ("ns3::GammaRandomVariable"),
                   MakePointerAccessor (&NakagamiPropagationLossModel::m_gammaRandomVariable),
                   MakePointerChecker<GammaRandomVariable> ());
  ;
  return tid;

}

NakagamiPropagationLossModel::NakagamiPropagationLossModel ()
{
}

double
NakagamiPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                             Ptr<MobilityModel> a,
                                             Ptr<MobilityModel> b) const
{
  // select m parameter

  double distance = a->GetDistanceFrom (b);
  NS_ASSERT (distance >= 0);

  double m;
  if (distance < m_distance1)
    {
      m = m_m0;
    }
  else if (distance < m_distance2)
    {
      m = m_m1;
    }
  else
    {
      m = m_m2;
    }

  // the current power unit is dBm, but Watt is put into the Nakagami /
  // Rayleigh distribution.
  double powerW = std::pow (10, (txPowerDbm - 30) / 10);

  double resultPowerW;

  // switch between Erlang- and Gamma distributions: this is only for
  // speed. (Gamma is equal to Erlang for any positive integer m.)
  unsigned int int_m = static_cast<unsigned int>(std::floor (m));

  if (int_m == m)
    {
      resultPowerW = m_erlangRandomVariable->GetValue (int_m, powerW / m);
    }
  else
    {
      resultPowerW = m_gammaRandomVariable->GetValue (m, powerW / m);
    }

  double resultPowerDbm = 10 * std::log10 (resultPowerW) + 30;

  NS_LOG_DEBUG ("Nakagami distance=" << distance << "m, " <<
                "power=" << powerW <<"W, " <<
                "resultPower=" << resultPowerW << "W=" << resultPowerDbm << "dBm");

  return resultPowerDbm;
}

int64_t
NakagamiPropagationLossModel::DoAssignStreams (int64_t stream)
{
  m_erlangRandomVariable->SetStream (stream);
  m_gammaRandomVariable->SetStream (stream + 1);
  return 2;
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (FixedRssLossModel);

TypeId 
FixedRssLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::FixedRssLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<FixedRssLossModel> ()
    .AddAttribute ("Rss", "The fixed receiver Rss.",
                   DoubleValue (-150.0),
                   MakeDoubleAccessor (&FixedRssLossModel::m_rss),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}
FixedRssLossModel::FixedRssLossModel ()
  : PropagationLossModel ()
{
}

FixedRssLossModel::~FixedRssLossModel ()
{
}

void
FixedRssLossModel::SetRss (double rss)
{
  m_rss = rss;
}

double
FixedRssLossModel::DoCalcRxPower (double txPowerDbm,
                                  Ptr<MobilityModel> a,
                                  Ptr<MobilityModel> b) const
{
  return m_rss;
}

int64_t
FixedRssLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (MatrixPropagationLossModel);

TypeId 
MatrixPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MatrixPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<MatrixPropagationLossModel> ()
    .AddAttribute ("DefaultLoss", "The default value for propagation loss, dB.",
                   DoubleValue (std::numeric_limits<double>::max ()),
                   MakeDoubleAccessor (&MatrixPropagationLossModel::m_default),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

MatrixPropagationLossModel::MatrixPropagationLossModel ()
  : PropagationLossModel (), m_default (std::numeric_limits<double>::max ())
{
}

MatrixPropagationLossModel::~MatrixPropagationLossModel ()
{
}

void 
MatrixPropagationLossModel::SetDefaultLoss (double loss)
{
  m_default = loss;
}

void
MatrixPropagationLossModel::SetLoss (Ptr<MobilityModel> ma, Ptr<MobilityModel> mb, double loss, bool symmetric)
{
  NS_ASSERT (ma != 0 && mb != 0);

  MobilityPair p = std::make_pair (ma, mb);
  std::map<MobilityPair, double>::iterator i = m_loss.find (p);

  if (i == m_loss.end ())
    {
      m_loss.insert (std::make_pair (p, loss));
    }
  else
    {
      i->second = loss;
    }

  if (symmetric)
    {
      SetLoss (mb, ma, loss, false);
    }
}

double 
MatrixPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                           Ptr<MobilityModel> a,
                                           Ptr<MobilityModel> b) const
{
  std::map<MobilityPair, double>::const_iterator i = m_loss.find (std::make_pair (a, b));

  if (i != m_loss.end ())
    {
      return txPowerDbm - i->second;
    }
  else
    {
      return txPowerDbm - m_default;
    }
}

int64_t
MatrixPropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (RangePropagationLossModel);

TypeId
RangePropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RangePropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<RangePropagationLossModel> ()
    .AddAttribute ("MaxRange",
                   "Maximum Transmission Range (meters)",
                   DoubleValue (250),
                   MakeDoubleAccessor (&RangePropagationLossModel::m_range),
                   MakeDoubleChecker<double> ())
  ;
  return tid;
}

RangePropagationLossModel::RangePropagationLossModel ()
{
}

double
RangePropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                          Ptr<MobilityModel> a,
                                          Ptr<MobilityModel> b) const
{
  double distance = a->GetDistanceFrom (b);
  if (distance <= m_range)
    {
      return txPowerDbm;
    }
  else
    {
      return -1000;
    }
}

int64_t
RangePropagationLossModel::DoAssignStreams (int64_t stream)
{
  return 0;
}

// ------------------------------------------------------------------------- //

} // namespace ns3
