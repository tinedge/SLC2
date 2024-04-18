/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Technische Universität Berlin
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
 */

#include "mygym.h"

#include "ns3/object.h"

#include "ns3/core-module.h"

#include <ns3/lte-module.h>

#include "ns3/node-list.h"

#include "ns3/log.h"

#include <sstream>

#include <iostream>

#include <ns3/lte-common.h>
#include "ns3/lte-enb-rrc.h"

#include <ns3/cell-individual-offset.h>

#include <stdlib.h>

#include <typeinfo>

#include <numeric>

#include <cmath>

namespace ns3 {

    NS_LOG_COMPONENT_DEFINE("MyGymEnv");

    NS_OBJECT_ENSURE_REGISTERED(MyGymEnv);

    MyGymEnv::MyGymEnv() {
        NS_LOG_FUNCTION(this);
    }

    MyGymEnv::MyGymEnv(double stepTime, uint32_t N1, uint32_t N2, uint16_t N3, uint32_t port) {
        NS_LOG_FUNCTION(this);
        collect = 0;
        collecting_window = 0.05; //50ms
        block_Thr = 0.5; // Blockage threshold 0.5 Mb/s
        m_chooseReward = 0; //0: average overall throughput, 1: PRBs utilization deviation, 2: number of blocked users
        m_interval = stepTime;
        m_cellCount = N1;
        m_userCount = N2;
        m_nRBTotal = N3 * collecting_window * 1000;
        m_port = port;
        m_rbUtil.assign(m_cellCount, 0);
        m_dlThroughput = 0;
        m_cellFrequency.assign(m_cellCount, 0);
        rewards.assign(m_cellCount, 0);
        rewards_sum.assign(m_cellCount, 0);
        m_dlThroughputVec.assign(m_cellCount, 0);
        m_UesNum.assign(m_cellCount, 0);
        std::vector < uint32_t > dummyVec(29, 0);
        m_MCSPen.assign(m_cellCount, dummyVec);
        Simulator::Schedule(Seconds(1), & MyGymEnv::ScheduleNextStateRead, this);
        Simulator::Schedule(Seconds(1 - collecting_window), & MyGymEnv::Start_Collecting, this);
        UserThrouput.clear();
    }

    void
    MyGymEnv::ScheduleNextStateRead() {
        NS_LOG_FUNCTION(this);
        Simulator::Schedule(Seconds(m_interval), & MyGymEnv::ScheduleNextStateRead, this);
        Notify();
    }
    void
    MyGymEnv::Start_Collecting() {
        NS_LOG_FUNCTION(this);
        collect = 1;
        NS_LOG_LOGIC("%%%%%%%% Start collecting %%%%%%%%  time= " << Simulator::Now().GetSeconds() << " sec");

    }
    MyGymEnv::~MyGymEnv() {
        NS_LOG_FUNCTION(this);
    }

    TypeId
    MyGymEnv::GetTypeId(void) {
        static TypeId tid = TypeId("MyGymEnv")
            .SetParent < OpenGymEnv > ()
            .SetGroupName("OpenGym")
            .AddConstructor < MyGymEnv > ();
        return tid;
    }

    void
    MyGymEnv::DoDispose() {
        NS_LOG_FUNCTION(this);
    }
    
    void 
    MyGymEnv::GetRlcStats(Ptr<RadioBearerStatsCalculator> m_rlcStats) {
        RlcStats = m_rlcStats;
    }
    
    void 
    MyGymEnv::AddNewNode(uint16_t cellId, Ptr<LteEnbNetDevice> dev){
        m_enbs.insert(std::pair<uint32_t, Ptr<LteEnbNetDevice>> (cellId, dev));
    }
    void 
    MyGymEnv::AddNewUe(uint64_t imsi, Ptr<LteUeNetDevice> dev){
        m_ues.insert(std::pair<uint64_t, Ptr<LteUeNetDevice>> (imsi, dev));
    }
    
    Ptr < OpenGymSpace >
        MyGymEnv::GetActionSpace() {
         
            uint32_t nodeNum = m_enbs.size();
            std::vector<uint32_t> shape {nodeNum,};
            std::vector<uint32_t> shape2 {m_userCount,};
            std::string dtype = TypeNameGet<float> ();

            Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace> (-6, 6, shape, dtype); //todo range change for -30 to 30
            

            NS_LOG_INFO("GetObservationSapce: "<<space);
            return space;
        }

    Ptr < OpenGymSpace >
        MyGymEnv::GetObservationSpace() {
            NS_LOG_FUNCTION(this);

            // Custom
            /////////////////////////////
            float low = -50.0;
            float high = 50.0;
            std::vector < uint32_t > shape = {m_cellCount,};
            std::string dtype = TypeNameGet < float > ();
            Ptr < OpenGymBoxSpace > rbUtil = CreateObject < OpenGymBoxSpace > (low, high, shape, dtype);
            Ptr < OpenGymBoxSpace > FarUes = CreateObject < OpenGymBoxSpace > (low, high, shape, dtype);
            Ptr < OpenGymBoxSpace > mroCount = CreateObject < OpenGymBoxSpace > (low, high, shape, dtype);
            Ptr < OpenGymBoxSpace > Throughput = CreateObject < OpenGymBoxSpace > (0, 50, shape, dtype);
            Ptr < OpenGymBoxSpace > dlPrbusage = CreateObject < OpenGymBoxSpace > (0, 1, shape, dtype);
            Ptr < OpenGymBoxSpace > AverageVelocity = CreateObject < OpenGymBoxSpace > (0, 15, shape, dtype);
            Ptr < OpenGymBoxSpace > AvgCqi = CreateObject < OpenGymBoxSpace > (0, 15, shape, dtype);
            Ptr < OpenGymBoxSpace > TotalCqi = CreateObject < OpenGymBoxSpace > (0, 50, shape, dtype);
            Ptr < OpenGymBoxSpace > enbMLBstate = CreateObject < OpenGymBoxSpace > (0, 8, shape, dtype); //Kihoon
            Ptr < OpenGymBoxSpace > MLBreward = CreateObject < OpenGymBoxSpace > (low, high, shape, dtype); //Kihoon
            Ptr < OpenGymBoxSpace > MROreward = CreateObject < OpenGymBoxSpace > (low, high, shape, dtype);
            Ptr < OpenGymBoxSpace > enbRlfNum = CreateObject < OpenGymBoxSpace > (low, high, shape, dtype);
            Ptr < OpenGymBoxSpace > enbPpNum = CreateObject < OpenGymBoxSpace > (low, high, shape, dtype);
            Ptr < OpenGymBoxSpace > enbBestCell = CreateObject < OpenGymBoxSpace > (low, high, shape, dtype);
            Ptr < OpenGymBoxSpace > enbStepPrb = CreateObject < OpenGymBoxSpace > (-100000, 100000, shape, dtype);
            Ptr < OpenGymBoxSpace > enbStepRlf = CreateObject < OpenGymBoxSpace > (-100000, 100000, shape, dtype);
            Ptr < OpenGymBoxSpace > enbStepPp = CreateObject < OpenGymBoxSpace > (-100000, 100000, shape, dtype);

            shape = {m_userCount,};
            Ptr < OpenGymBoxSpace > CurrentRsrp = CreateObject < OpenGymBoxSpace > (-150, 100000, shape, dtype);
            Ptr < OpenGymBoxSpace > Reward = CreateObject < OpenGymBoxSpace > (-5, 5, shape, dtype);
            Ptr < OpenGymBoxSpace > AverageSinr = CreateObject < OpenGymBoxSpace > (-100000, 100000, shape, dtype);
            Ptr < OpenGymBoxSpace > EnbDisance = CreateObject < OpenGymBoxSpace > (-100000, 100000, shape, dtype);
            Ptr < OpenGymBoxSpace > IsBestCell = CreateObject < OpenGymBoxSpace > (-100000, 100000, shape, dtype);
            Ptr < OpenGymBoxSpace > Results = CreateObject < OpenGymBoxSpace > (-100000, 100000, shape, dtype);

            Ptr<OpenGymDictSpace> space = CreateObject<OpenGymDictSpace> ();
            space -> Add("rbUtil", rbUtil);
            space -> Add("FarUes", FarUes);
            space -> Add("mroCount", mroCount);
            space -> Add("AvgCqi", AvgCqi);
            space -> Add("TotalCqi",TotalCqi);
            space -> Add("dlPrbusage", dlPrbusage);
            space -> Add("CurrentRsrp",CurrentRsrp);
            space -> Add("Reward",Reward);
            space -> Add("AverageSinr",AverageSinr);
            space -> Add("EnbDistance",EnbDisance);
            space -> Add("IsBestCell",IsBestCell);
            space -> Add("MROreward",MROreward);
            space -> Add("Results",Results);
            space -> Add("AverageVelcity",AverageVelocity);
            space -> Add("enbMLBstate", enbMLBstate);
            space -> Add("MLBreward", MLBreward);
            space -> Add("enbRlfNum", enbRlfNum);
            space -> Add("enbPpNum", enbPpNum);
            space -> Add("enbBestCell",enbBestCell);
            space -> Add("enbStepPrb",enbStepPrb);
            space -> Add("enbStepRlf",enbStepRlf);
            space -> Add("enbStepPp",enbStepPp);

            return space;
        }

    bool
    MyGymEnv::GetGameOver() {
        NS_LOG_FUNCTION(this);
        bool isGameOver = false;
        NS_LOG_LOGIC("MyGetGameOver: " << isGameOver);
        return isGameOver;
    }

    Ptr < OpenGymDataContainer >
        MyGymEnv::GetObservation() {
            NS_LOG_FUNCTION(this);
            // Custom
            /////////////////////////////
            calculate_rewards();

            step ++;

            Ptr < OpenGymDictContainer > obsContainer = CreateObject < OpenGymDictContainer > ();
            std::vector < uint32_t > shape = { m_cellCount, };

            std::vector<float> Far_UEs;
            std::vector<float> Served_UEs;

            std::vector<float> normal_RB;
            // std::vector<float> PRB_lin;

            Ptr < OpenGymBoxContainer < float > > box1 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box1 = CreateObject < OpenGymBoxContainer < float > > (shape);

            std::map<uint8_t, float> enbStepRbUtil;
            float normalizedRBUtil;
            for (uint8_t idx = 0; idx < m_cellCount; ++idx) {
                normalizedRBUtil = float(m_rbUtil.at(idx)) / m_nRBTotal ;
                normal_RB.push_back(normalizedRBUtil);
                box1 -> AddValue(normalizedRBUtil);
                //To see normalized RB util
                enbStepRbUtil[idx+1] = normalizedRBUtil;
            }
            obsContainer -> Add("rbUtil", box1);

            uint32_t enbIdx = 1;
            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter != m_enbs.end(); ++iter){
                enbIdx ++;
            }

            // For MRO State
            std::vector<float> SinrAvg;
            std::vector<float> IsBestCell;
            for (std::map<uint64_t, Ptr<LteUeNetDevice>>::iterator iter = m_ues.begin(); iter != m_ues.end(); ++iter)
            {
                double sinrAvg = iter->second->GetPhy()->GetAverageSinr();
                double isBestCell = iter->second->GetPhy()->GetIsBestcell();

                sinrAvg *= 10000;
                sinrAvg = round(sinrAvg);
                sinrAvg /= 10000;

                isBestCell = round(isBestCell);


                SinrAvg.push_back(sinrAvg);
                IsBestCell.push_back(isBestCell);
            }
            
            // Average SINR
            Ptr < OpenGymBoxContainer < float > > box2 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box2 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box2 -> SetData(SinrAvg);
            obsContainer -> Add("AverageSinr",box2);
            // IsBestCell
            Ptr < OpenGymBoxContainer < float > > box3 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box3 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box3 -> SetData(IsBestCell);
            obsContainer -> Add("IsBestCell",box3);

            Ptr < OpenGymBoxContainer < float > > box4 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box4 = CreateObject < OpenGymBoxContainer < float > > (shape);
            for (std::map<uint64_t, Ptr<LteUeNetDevice>>::iterator iter = m_ues.begin(); iter != m_ues.end(); ++iter){
                double currentRsrp = iter->second->GetPhy()->GetCurrentRsrp();

                currentRsrp /= 1000;

                box4 -> AddValue(currentRsrp);
            }
            obsContainer -> Add("CurrentRsrp",box4);

            // Average CQI
            Ptr < OpenGymBoxContainer < float > > box5 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box5 = CreateObject < OpenGymBoxContainer < float > > (shape);

            std::vector <float> TotalCqi(m_cellCount);
            float CqiSum = 0;

            std::map<uint32_t, float> enbStepCqi;
            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter != m_enbs.end(); ++iter){
                uint32_t CellId = iter->first;
                std::vector<uint64_t> Imsi_List;

                std::map<uint16_t, Ptr<UeManager>> UeMap = iter->second->GetRrc ()->m_ueMap;

                for (std::map<uint16_t, Ptr<UeManager>>::iterator iter2 = UeMap.begin(); iter2 != UeMap.end(); ++iter2){
                    uint64_t Imsi = iter2->second->GetImsi();
                    Imsi_List.push_back(Imsi);
                }

                float AvgCqi = 0;
                for (uint64_t i=0; i<Imsi_List.size(); i++)
                {
                    float UeCqi = float(m_ues[Imsi_List[i]]->GetPhy()->AvgCqi);
                    AvgCqi += UeCqi;
                }
                
                if(Imsi_List.size() == 0){
                    AvgCqi = 0;
                    
                    CqiSum = 0;
                }
                else{
                    CqiSum += AvgCqi;
                    
                    AvgCqi = AvgCqi / Imsi_List.size(); 
                }

                AvgCqi = round(AvgCqi * 100) / 100;

                enbStepCqi[CellId] = AvgCqi;

                box5 -> AddValue(AvgCqi);
            }
            obsContainer -> Add("AvgCqi", box5);

            Ptr < OpenGymBoxContainer < float > > box6 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box6 = CreateObject < OpenGymBoxContainer < float > > (shape);
            CqiSum = CqiSum / m_ues.size();
            for (uint8_t idx = 0; idx < m_cellCount; ++idx) {
                TotalCqi[idx] = CqiSum;
            }
            box6 -> SetData(TotalCqi);
            obsContainer -> Add("TotalCqi",box6);
            
            ///////////////////////////

            // Far UEs, Served UEs
            std::map<uint16_t, double> distanceMap; // For MRO state

            double TotalFarUes = 0;

            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter != m_enbs.end(); ++iter){
                // uint32_t CellId = iter->first;
                std::vector<uint64_t> Imsi_List;
                double eNB_x = iter->second->GetNode()->GetObject<MobilityModel>()->GetPosition().x;
                double eNB_y = iter->second->GetNode()->GetObject<MobilityModel>()->GetPosition().y;
                double eNB_z = iter->second->GetNode()->GetObject<MobilityModel>()->GetPosition().z;

                std::map<uint16_t, Ptr<UeManager>> UeMap = iter->second->GetRrc ()->m_ueMap;
                
                for (std::map<uint16_t, Ptr<UeManager>>::iterator iter2 = UeMap.begin(); iter2 != UeMap.end(); ++iter2){
                    uint64_t Imsi = iter2->second->GetImsi();
                    Imsi_List.push_back(Imsi);
                }

                double distance_sum = 0;
                double far_distance_ues = 0;
                for (uint64_t i=0; i<Imsi_List.size(); i++){
                    Ptr<LteUeNetDevice> UeNetDevice = m_ues[Imsi_List[i]];
                    double Ue_x = UeNetDevice->GetNode()->GetObject<MobilityModel>()->GetPosition().x;
                    double Ue_y = UeNetDevice->GetNode()->GetObject<MobilityModel>()->GetPosition().y;
                    double Ue_z = UeNetDevice->GetNode()->GetObject<MobilityModel>()->GetPosition().z;

                    double distance = sqrt( pow(eNB_x-Ue_x, 2) + pow(eNB_y-Ue_y, 2) + pow(eNB_z-Ue_z, 2) );
                    distance_sum += distance;

                    // if (distance >= 237.74){
                    if (distance >= 137.74){
                        far_distance_ues += 1;

                        //
                        TotalFarUes += 1;
                    }
                    // For MRO State
                    distanceMap[Imsi_List[i]] = distance;
                }
                
                // double distance_avg;
                float ratio_far_ues;
                float ratio_ues;
                float served_ues;

                if (Imsi_List.size() == 0){
                    // distance_avg = 0;
                    ratio_far_ues = 0;
                    ratio_ues = 0;
                    served_ues = 0;
                }
                else {
                    // distance_avg = double(distance_sum) / double(Imsi_List.size());
                    ratio_far_ues = float( far_distance_ues / Imsi_List.size() );
                    ratio_ues = float(Imsi_List.size()) / m_userCount;
                    served_ues = float(Imsi_List.size());
                }
                
                ratio_ues = round(ratio_ues * 100) / 100;
                ratio_far_ues = round(ratio_far_ues * 100) / 100;
                
                Served_UEs.push_back(served_ues);
                Far_UEs.push_back(ratio_far_ues);
            }

            double TotalFarUesRatio = double(TotalFarUes/m_userCount);

            // For Step RLF, PP in eNB
            std::map<uint32_t, int> eNBStepRlf;
            std::map<uint32_t, int> eNBStepPp; 

            std::map<uint32_t, float> eNBStepMroReward;

            float TotalMroReward =0;

            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter != m_enbs.end(); ++iter){
                std::vector<uint64_t> Imsi_List;
                uint32_t CellId = iter->first;
                std::map<uint16_t, Ptr<UeManager>> UeMap = iter->second->GetRrc ()->m_ueMap;
                
                for (std::map<uint16_t, Ptr<UeManager>>::iterator iter2 = UeMap.begin(); iter2 != UeMap.end(); ++iter2){
                    uint64_t Imsi = iter2->second->GetImsi();
                    Imsi_List.push_back(Imsi);
                }

                int stepRlf = 0;
                int stepPp = 0;
                float stepReward = 0;

                for (uint64_t i=0; i<Imsi_List.size(); i++){
                    Ptr<LteUeNetDevice> UeNetDevice = m_ues[Imsi_List[i]];
                    
                    // Step RLF
                    int Counter1 = UeNetDevice->GetPhy()->GetTooLateHO_CNT();
                    int Counter2 = UeNetDevice->GetPhy()->GetTooEarlyHO_CNT();
                    int Counter3 = UeNetDevice->GetPhy()->GetWrongCellHO_CNT();

                    stepRlf = stepRlf + (Counter1 + Counter2 + Counter3);

                    // Step PP
                    int Counter4 = UeNetDevice->GetPhy()->GetPingPong_CNT();

                    stepPp = stepPp + Counter4;

                    stepReward = stepReward -(0.3*Counter4 + 0.1*Counter3 + 0.2*Counter2 + 0.5*Counter1);
                }
                
                eNBStepRlf[CellId] = stepRlf;
                eNBStepPp[CellId] = stepPp;
                eNBStepMroReward[CellId] = stepReward;

                TotalMroReward = TotalMroReward - stepReward;
                
            }

            // For MRO State
            std::vector<float> distanceVector;
            for (uint32_t i=1; i<(m_userCount+1); ++i){
                                double distance = distanceMap[i];
                distance *= 10000;
                distance = round(distance);
                distance /= 10000;
                distanceVector.push_back(distance);

            }

            Ptr < OpenGymBoxContainer < float > > box7 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box7 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box7 -> SetData(distanceVector);
            obsContainer -> Add("EnbDistance", box7);

            Ptr < OpenGymBoxContainer < float > > box8 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box8 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box8 -> SetData(Far_UEs);
            obsContainer -> Add("FarUes", box8);

            // DL Throughput
            dlThroughput_IMSI = RlcStats->GetdlThroughput_IMSI();
      
            std::vector <float> throughput(m_cellCount);
            for (std::map<uint64_t, Ptr<LteUeNetDevice>>::iterator iter = m_ues.begin(); iter != m_ues.end(); ++iter)
            {
                uint16_t CellId = iter->second->GetRrc()->GetCellId();
                uint32_t dlThroughput = dlThroughput_IMSI[iter->first];
                throughput[CellId-1] = throughput[CellId-1] + ( double(dlThroughput) * 8.0 / 1000000.0);


            }
            // float throughput_total = 0.0;
            // throughput_total = std::accumulate(throughput.begin (), throughput.end (), 0.0);
            
            Ptr < OpenGymBoxContainer < float > > box9 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box9 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box9 -> SetData(throughput);
            obsContainer -> Add("Throughput", box9);

            // RLF Counter Test
            ////////////////////////
            int Case1_Counter = 0;
            int Case2_Counter = 0;
            int Case3_Counter = 0;
            int Case4_Counter = 0; //kihoon

            double ThroughputSum = 0;
            int CurrentRlfNum = 0;
            int CurrentPpNum = 0;

            for (std::map<uint64_t, Ptr<LteUeNetDevice>>::iterator iter = m_ues.begin(); iter != m_ues.end(); ++iter)
            {   
                // Step RLF
                int Counter1 = iter->second->GetPhy()->GetTooLateHO_CNT();
                int Counter2 = iter->second->GetPhy()->GetTooEarlyHO_CNT();
                int Counter3 = iter->second->GetPhy()->GetWrongCellHO_CNT();

                // Step PP
                int Counter4 = iter->second->GetPhy()->GetPingPong_CNT();//kihoon

                uint32_t dlThroughput = dlThroughput_IMSI[iter->first];

                RLF_Counter += Counter1 + Counter2 +Counter3;
                Pingpong_Counter += Counter4;
                
                iter->second->GetPhy()->ClearTooLateHO_CNT();
                iter->second->GetPhy()->ClearTooEarlyHO_CNT();
                iter->second->GetPhy()->ClearWrongCellHO_CNT();
                iter->second->GetPhy()->ClearPingPong_CNT();//kihoon
                
               
                Case1_Counter += Counter1;
                Case2_Counter += Counter2;
                Case3_Counter += Counter3;
                Case4_Counter += Counter4;//kihoon
                
                double UeThroughput = ( double(dlThroughput) * 8.0 / 1000000.0 );

                ThroughputSum = ThroughputSum + UeThroughput;
                CurrentRlfNum = CurrentRlfNum+ Counter1 + Counter2 +Counter3;
                CurrentPpNum = CurrentPpNum + Counter4;
            }

            double RlfRate;
            double PpRate;
            if (CurrentRlfNum == 0){
                RlfRate = 0;
            }
            else{
                RlfRate = double(CurrentRlfNum)/10;
            }
            if (CurrentPpNum == 0){
                PpRate = 0;
            }
            else{
                PpRate = double(CurrentPpNum)/10; 
            }
            
            double AverageThroughput = ThroughputSum/m_userCount;

            double Reward = 0.4*AverageThroughput - (0.4*RlfRate + 0.2*PpRate);

            Reward *= 10000;
            Reward = round(Reward);
            Reward /= 10000;
            
            Ptr < OpenGymBoxContainer < float > > box10 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box10 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box10->AddValue(Reward);
            obsContainer -> Add("Reward",box10);

            Ptr < OpenGymBoxContainer < double > > box11 = CreateObject < OpenGymBoxContainer < double > > (shape);
            box11 = CreateObject < OpenGymBoxContainer < double > > (shape);
            std::vector <double> MroRewardVector;
            for (std::map<uint32_t, float>::iterator iter = eNBStepMroReward.begin(); iter != eNBStepMroReward.end(); ++iter){
                
                double MroReward = iter->second;

                MroRewardVector.push_back(MroReward);
            }
            box11 -> SetData(MroRewardVector);
            obsContainer -> Add("MROreward",box11);

            ////////////////////////

            // RLF Test
            ////////////////////////
            std::vector<float> mroCount(9, 0);
            std::map<uint16_t,int> RlfCounter;
            for (std::map<uint64_t, Ptr<LteUeNetDevice>>::iterator iter = m_ues.begin(); iter != m_ues.end(); ++iter)
            {
                std::map<uint16_t,int> RlfCounterperUe = iter->second->GetPhy()->GetRlfCounter();
                iter->second->GetPhy()->ClearRlfCounter();
            for (auto iter2 =RlfCounterperUe.begin();iter2!=RlfCounterperUe.end();iter2++)
            {
                if(RlfCounter.count(iter2->first)){
                    RlfCounter[iter2->first]+=iter2->second;
                }
                else{
                    RlfCounter.insert({iter2->first,iter2->second});
                }
            }
            }
            for (auto iter3 = RlfCounter.begin(); iter3 != RlfCounter.end(); iter3++)
            {
                if (iter3->first >= 1 && iter3->first <= mroCount.size()){
                        mroCount[iter3->first - 1] = iter3->second;
                }

            }
            for (int k =0; k != 9 ; k++ )
            {
               if (mroCount[k]>0){
                   
                   mroCount[k]= 8;
               }
               else if (mroCount[k]==0){
                   mroCount[k] = 0;
               }
               else {
                   mroCount[k] = -8;
               }

            } //kihoon 0517
            
            Ptr < OpenGymBoxContainer < float > > box12 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box12 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box12 -> SetData(mroCount);
            obsContainer -> Add("mroCount", box12);
            ////////////////////////


            NS_LOG_LOGIC("MyGetObservation: " << obsContainer);
             double ueNum;
            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter =m_enbs.begin(); iter != m_enbs.end(); ++iter){
            ueNum += (double)(iter->second->GetRrc ()->m_numofues);
                 }

            Ptr < OpenGymBoxContainer < double > > box13 = CreateObject < OpenGymBoxContainer < double > > (shape);
            box13 = CreateObject < OpenGymBoxContainer < double > > (shape);

            std::map<uint32_t, double> enbStepPrb;
            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
                uint32_t CellId = iter->first;
                double numofues;
                double dlprbusage;
                numofues = (double)(iter->second->GetRrc ()->m_numofues);
                if(ueNum==0){
                    dlprbusage = 0;
                }
                else{
                    dlprbusage = (numofues/ueNum)*(double)100;
                }
                dlprbusage = (round(dlprbusage*100))/100;
                box13->AddValue(dlprbusage);

                enbStepPrb[CellId] = dlprbusage;

            }
            obsContainer -> Add("dlPrbusage", box13);



            // Average Velocity
            Ptr < OpenGymBoxContainer < float > > box14 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box14 = CreateObject < OpenGymBoxContainer < float > > (shape);

            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter != m_enbs.end(); ++iter){
                
                // uint32_t CellId = iter->first;
                std::vector<uint64_t> Imsi_List;

                std::map<uint16_t, Ptr<UeManager>> UeMap = iter->second->GetRrc ()->m_ueMap;
                
                for (std::map<uint16_t, Ptr<UeManager>>::iterator iter2 = UeMap.begin(); iter2 != UeMap.end(); ++iter2){
                    uint64_t Imsi = iter2->second->GetImsi();
                    Imsi_List.push_back(Imsi);
                }
                double SumVelocity = 0.0;
                double NumofUe = Imsi_List.size();
                for (uint64_t i=0; i<Imsi_List.size(); i++){
                    Ptr<LteUeNetDevice> UeNetDevice = m_ues[Imsi_List[i]];

                    double velocity_x = UeNetDevice->GetNode()->GetObject<MobilityModel>()->GetVelocity().x;
                    double velocity_y = UeNetDevice->GetNode()->GetObject<MobilityModel>()->GetVelocity().y;
                    double velocity_z = UeNetDevice->GetNode()->GetObject<MobilityModel>()->GetVelocity().z;

                    double velocity = sqrt( pow(velocity_x, 2) + pow(velocity_y, 2) + pow(velocity_z, 2) );

                    SumVelocity = SumVelocity + velocity;
                }

                double AverageVelocity;
                if (Imsi_List.size()==0){
                    AverageVelocity = 0;
                }
                else{
                    AverageVelocity = SumVelocity / NumofUe;
                }


                double MappedVelocity;

                if(m_port==1167 || m_port == 1403){
                    if(AverageVelocity>=0 && AverageVelocity<=9){
                    MappedVelocity = 0;
                    }
                    else if(AverageVelocity>9 && AverageVelocity<=15){
                        MappedVelocity = 1;
                    }
                    else if(AverageVelocity>15 && AverageVelocity<=20){
                        MappedVelocity = 2;
                    }
                    else if(AverageVelocity>20 && AverageVelocity<=25){
                        MappedVelocity = 3;
                    }
                    else if(AverageVelocity>25 && AverageVelocity<=30){
                        MappedVelocity = 4;
                    }
                    else if(AverageVelocity>30 && AverageVelocity<=35){
                        MappedVelocity = 5;
                    }
                    else if(AverageVelocity>35 && AverageVelocity<=40){
                        MappedVelocity = 6;
                    }
                    else if(AverageVelocity>40 && AverageVelocity<=45){
                        MappedVelocity = 7;
                    }
                    else if(AverageVelocity>45 && AverageVelocity<=50){
                        MappedVelocity = 8;
                    }
                    else if(AverageVelocity>50 && AverageVelocity<=55){
                        MappedVelocity = 9;
                    }
                    else if(AverageVelocity>55 && AverageVelocity<=60){
                        MappedVelocity = 10;
                    }
                    else{
                        MappedVelocity = 0;
                    }
                }

                box14 ->AddValue(MappedVelocity);
            }
            obsContainer -> Add("AverageVelocity",box14);

            std::map<uint32_t, double> enbMLBindicator; 
            float CriticalCellLoad = 0;
            float CriticalCellLoad2 = 0;
            float CriticalCellLoad4 = 0;
            float CriticalCellLoad6 = 0;
            float CriticalCellLoad8 = 0;
            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
                uint32_t CellId = iter->first;

                if(CellId==5){
                    CriticalCellLoad = enbStepPrb[CellId];
                }
                if(CellId==2){
                    CriticalCellLoad2 = enbStepPrb[CellId];
                }
                if(CellId==4){
                    CriticalCellLoad4 = enbStepPrb[CellId];
                }
                if(CellId==6){
                    CriticalCellLoad6 = enbStepPrb[CellId];
                }
                if(CellId==8){
                    CriticalCellLoad8 = enbStepPrb[CellId];
                }
                
                double MLBindicator = 0;
                if(enbStepCqi[CellId] < 5)
                    MLBindicator = 2.1*enbStepCqi[CellId] + enbStepPrb[CellId] + enbStepRbUtil[CellId];
                else if (5 <= enbStepCqi[CellId] && enbStepCqi[CellId] <= 7)
                    MLBindicator = 1.1*enbStepCqi[CellId] + enbStepPrb[CellId] + enbStepRbUtil[CellId];
                else 
                    MLBindicator = 0.1*enbStepCqi[CellId] + enbStepPrb[CellId] + enbStepRbUtil[CellId];


                if(m_port == 1403){
                    MLBindicator = MLBindicator * (1.0 / 2.0);
                }

                enbMLBindicator[CellId]= MLBindicator;
                box2->AddValue(MLBindicator);
            }

            std::map<uint32_t, double> enbneigborMLBindicator;
            // For Large scale
            if (m_port == 1167) { 
            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
                uint32_t CellId = iter->first;

                double neigborMLBindicator = 0;
                if (CellId == 0){
                    neigborMLBindicator =  (enbMLBindicator[1] + enbMLBindicator[3] + enbMLBindicator[4])/3;
                }
                else if (CellId == 1){
                    neigborMLBindicator =  (enbMLBindicator[0] + enbMLBindicator[2] + enbMLBindicator[3] 
                    + enbMLBindicator[4] +enbMLBindicator[5])/5;
                }
                else if (CellId == 2){
                    neigborMLBindicator =  (enbMLBindicator[1] + enbMLBindicator[4] + enbMLBindicator[5])/3;
                }
                else if (CellId == 3){
                    neigborMLBindicator =  (enbMLBindicator[0] + enbMLBindicator[1] + enbMLBindicator[4]
                    +enbMLBindicator[6] + enbMLBindicator[7])/5;
                }
                else if (CellId == 4){
                    neigborMLBindicator =  (enbMLBindicator[0] + enbMLBindicator[1] + enbMLBindicator[2]
                    +enbMLBindicator[3] + enbMLBindicator[5]+ enbMLBindicator[6] + enbMLBindicator[7] + enbMLBindicator[8])/8;
                }
                else if (CellId == 5){
                    neigborMLBindicator =  (enbMLBindicator[1] + enbMLBindicator[2] + enbMLBindicator[4]
                    +enbMLBindicator[7] + enbMLBindicator[8])/5;
                }
                else if (CellId == 6){
                    neigborMLBindicator =  (enbMLBindicator[3] + enbMLBindicator[4] + enbMLBindicator[7])/3;
                }
                else if (CellId == 7){
                    neigborMLBindicator =  (enbMLBindicator[3] + enbMLBindicator[4] + enbMLBindicator[5]
                    +enbMLBindicator[6] + enbMLBindicator[8])/5;
                }
                else {
                    neigborMLBindicator =  (enbMLBindicator[4] + enbMLBindicator[5] + enbMLBindicator[7])/3;
                }
                enbneigborMLBindicator[CellId] =neigborMLBindicator;

            }
            }

            // For small scale
            if(m_port == 1403){
            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
                uint32_t CellId = iter->first;

                double neigborMLBindicator = 0;
                if (CellId == 0){
                    neigborMLBindicator =  (enbMLBindicator[1] + enbMLBindicator[2] + enbMLBindicator[3])/3;
                }
                else if (CellId == 1){
                    neigborMLBindicator =  (enbMLBindicator[0] + enbMLBindicator[2] + enbMLBindicator[4])/3;
                }
                else if (CellId == 2){
                    neigborMLBindicator =  (enbMLBindicator[0] + enbMLBindicator[1] + enbMLBindicator[3]+ enbMLBindicator[4])/4;
                }
                else if (CellId == 3){
                    neigborMLBindicator =  (enbMLBindicator[0] + enbMLBindicator[2] + enbMLBindicator[4])/3;
                }
                else {
                    neigborMLBindicator =  (enbMLBindicator[1] + enbMLBindicator[2] + enbMLBindicator[3])/3;
                }
                enbneigborMLBindicator[CellId] =neigborMLBindicator;
             }
            }

            std::map<uint32_t, float> enbMLBstate; 
            Ptr < OpenGymBoxContainer < float > > box17 = CreateObject < OpenGymBoxContainer < float > > (shape);
            box17 = CreateObject < OpenGymBoxContainer < float > > (shape);
            
            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
            uint32_t CellId = iter->first;
            uint32_t MLBstate;
            
            if(enbMLBindicator[CellId] < 8){
                if (enbneigborMLBindicator[CellId]<8){
                    MLBstate = 0;
                }
                else if (8<= enbneigborMLBindicator[CellId] && enbneigborMLBindicator[CellId]<=14){
                    MLBstate = 1;
                }
                else{
                    MLBstate = 2;
                }
            }

            else if(8<= enbMLBindicator[CellId] && enbMLBindicator[CellId] < 15){
                if (enbneigborMLBindicator[CellId]<8){
                    MLBstate = 3;
                }
                else if (8<= enbneigborMLBindicator[CellId] && enbneigborMLBindicator[CellId]<=14){
                    MLBstate = 4;
                }
                else{
                    MLBstate = 5;
                }
            }
            
            else if(15<= enbMLBindicator[CellId] && enbMLBindicator[CellId] < 24){
                if (enbneigborMLBindicator[CellId]<8){
                    MLBstate = 6;
                }
                else if (8<= enbneigborMLBindicator[CellId] && enbneigborMLBindicator[CellId]<=14){
                    MLBstate = 7;
                }
                else{
                    MLBstate = 8;
                }
            }

             else if(24 <= enbMLBindicator[CellId]){
                if (enbneigborMLBindicator[CellId]<8){
                    MLBstate = 9;
                }
                else if (8<= enbneigborMLBindicator[CellId] && enbneigborMLBindicator[CellId]<=14){
                    MLBstate = 10;
                }
                else{
                    MLBstate = 11;
                }
            }
            
            if(30 <= Far_UEs[CellId] && Far_UEs[CellId] < 60 ){
                MLBstate +=12;
            }
            else if (60 <=Far_UEs[CellId]){
                MLBstate +=24;
            }
        
            enbMLBstate[CellId] = MLBstate;
            box17->AddValue(MLBstate);
            
        }
            
        obsContainer -> Add("enbMLBstate",box17);
    

        Ptr < OpenGymBoxContainer < double > > box18 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box18 = CreateObject < OpenGymBoxContainer < double > > (shape);
        std::vector<double> enbMLBreward;
        for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
            uint32_t CellId = iter->first;

            double MLBreward;

            if(step == 1){
                MLBreward = 0;
                enbMLBreward.push_back(MLBreward);
                preenbMLBstate[CellId] = enbMLBstate[CellId];
                preenbneigborMLBindicator[CellId] = enbneigborMLBindicator[CellId];
            }
            else{
                if (0 <= ( abs(enbneigborMLBindicator[CellId] - preenbneigborMLBindicator[CellId]) ) && ( abs(enbneigborMLBindicator[CellId] - preenbneigborMLBindicator[CellId]) )< 8){

                    if((enbMLBstate[CellId]-preenbMLBstate[CellId]) <= -2)
                        MLBreward = -2;
                    else if ((enbMLBstate[CellId]-preenbMLBstate[CellId]) == -1)
                        MLBreward = -1;
                    else if ((enbMLBstate[CellId]-preenbMLBstate[CellId]) == 0)
                        MLBreward = 0;
                    else if ((enbMLBstate[CellId]-preenbMLBstate[CellId]) == 1)
                        MLBreward = 1;    
                    else  
                        MLBreward = 2;   
                    
                    // Small
                    if(m_port == 1403){
                        if((enbMLBstate[CellId]<6) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) >=2)
                            MLBreward = 2;
                        else if ((enbMLBstate[CellId]<5) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) == 1)
                            MLBreward = 1;
                        else if ((enbMLBstate[CellId]<5) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) == 0)
                            MLBreward = 0;
                        else if ((enbMLBstate[CellId]<5) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) == -1)
                            MLBreward = -2;
                        else if ((enbMLBstate[CellId]<5) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) <= -2)
                            MLBreward = -3;
                    }

                }        
                else if (8 <= ( abs(enbneigborMLBindicator[CellId] - preenbneigborMLBindicator[CellId]) )){

                    if((enbMLBstate[CellId]-preenbMLBstate[CellId]) <= -2)
                        MLBreward = -3;
                    else if ((enbMLBstate[CellId]-preenbMLBstate[CellId]) == -1)
                        MLBreward = -2;
                    else if ((enbMLBstate[CellId]-preenbMLBstate[CellId]) == 0)
                        MLBreward = -1;
                    else if ((enbMLBstate[CellId]-preenbMLBstate[CellId]) == 1)
                        MLBreward = 0;    
                    else  
                        MLBreward = 1;   

                    // Small
                    if(m_port == 1403){

                        if((enbMLBstate[CellId]<5) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) >=2)
                            MLBreward = 3;
                        else if ((enbMLBstate[CellId]<5) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) == 1)
                            MLBreward = 2;
                        else if ((enbMLBstate[CellId]<5) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) == 0)
                            MLBreward = 0;
                        else if ((enbMLBstate[CellId]<5) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) == -1)
                            MLBreward = -1;
                        else if ((enbMLBstate[CellId]<5) && (enbMLBstate[CellId]-preenbMLBstate[CellId]) <= -2)
                            MLBreward = -2;
                    }

                }       
                enbMLBreward.push_back(MLBreward);
                preenbMLBstate[CellId] = enbMLBstate[CellId];
                preenbneigborMLBindicator[CellId] = enbneigborMLBindicator[CellId];
            }  

            box18->AddValue(MLBreward);
        }
        obsContainer -> Add("MLBreward",box18);


        // coordinator reward 
        Ptr < OpenGymBoxContainer < double > > box19 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box19 = CreateObject < OpenGymBoxContainer < double > > (shape);

        float steptotalRlf;
        float steptotalPp;
        for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
          uint32_t CellId = iter->first;
        
          steptotalPp+=eNBStepPp[CellId];
          steptotalRlf+=eNBStepRlf[CellId];

        }
        
        float MROimpact = 0;
        int totalStepRlf = 0;
        int totalStepPp = 0;
        for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
          uint32_t CellId = iter->first;   
          int a = eNBStepRlf[CellId];   
          int b = eNBStepPp[CellId];

          if(std::isnan(a)){
            a = 0;
          }
          if(std::isnan(b)){
            b = 0;
          }

          totalStepRlf = totalStepRlf + a;
          totalStepPp = totalStepPp +b;
        }

        MROimpact = 2*float(totalStepRlf) + float(totalStepPp);

        if(std::isnan(MROimpact)){
            MROimpact = 0;
        }       

        float Mean = 0;
        float sum = 0;
        for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
          uint32_t CellId = iter->first;      
          sum +=enbMLBindicator[CellId];
        }
        Mean = sum/m_enbs.size();

        float MLBimpact = 0;
        float std_sum = 0;


        for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
          uint32_t CellId = iter->first;  
          std_sum=+pow(Mean-enbMLBindicator[CellId],2);
        }
        MLBimpact = sqrt(std_sum);

        if(std::isnan(MLBimpact)){
            MLBimpact = 0;
        }

        float reward_coordinator;

        reward_coordinator = -(2.75* MROimpact+ MLBimpact); 

        box19->AddValue(reward_coordinator);
        obsContainer -> Add("reward_coordinator",box19);

        Ptr < OpenGymBoxContainer < double > > box20 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box20 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box20->AddValue(RLF_Counter);
        box20->AddValue(Pingpong_Counter);
        box20->AddValue(TotalMroReward);
        box20->AddValue(CriticalCellLoad);
        box20->AddValue(TotalFarUesRatio);

        box20->AddValue(CriticalCellLoad2);
        box20->AddValue(CriticalCellLoad4);
        box20->AddValue(CriticalCellLoad6);
        box20->AddValue(CriticalCellLoad8);

        obsContainer ->Add("Results",box20);

        Ptr < OpenGymBoxContainer < double > > box21 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box21 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box21->AddValue(eNBStepRlf[0]);
        box21->AddValue(eNBStepRlf[1]);
        box21->AddValue(eNBStepRlf[2]);
        box21->AddValue(eNBStepRlf[3]);
        box21->AddValue(eNBStepRlf[4]);
        box21->AddValue(eNBStepRlf[5]);
        box21->AddValue(eNBStepRlf[6]);
        box21->AddValue(eNBStepRlf[7]);
        box21->AddValue(eNBStepRlf[8]);

        obsContainer -> Add("enbRlfNum", box21);

        Ptr < OpenGymBoxContainer < double > > box22 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box22 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box22->AddValue(eNBStepPp[0]);
        box22->AddValue(eNBStepPp[1]);
        box22->AddValue(eNBStepPp[2]);
        box22->AddValue(eNBStepPp[3]);
        box22->AddValue(eNBStepPp[4]);
        box22->AddValue(eNBStepPp[5]);
        box22->AddValue(eNBStepPp[6]);
        box22->AddValue(eNBStepPp[7]);
        box22->AddValue(eNBStepPp[8]);

        obsContainer -> Add("enbPpNum", box22);

        Ptr < OpenGymBoxContainer < double > > box23 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box23 = CreateObject < OpenGymBoxContainer < double > > (shape);

        for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter != m_enbs.end(); ++iter){
                std::vector<uint64_t> Imsi_List;
                // uint32_t CellId = iter->first;
                std::map<uint16_t, Ptr<UeManager>> UeMap = iter->second->GetRrc ()->m_ueMap;
                
                for (std::map<uint16_t, Ptr<UeManager>>::iterator iter2 = UeMap.begin(); iter2 != UeMap.end(); ++iter2){
                    uint64_t Imsi = iter2->second->GetImsi();
                    Imsi_List.push_back(Imsi);
                }

                double isBestCell;

                for (uint64_t i=0; i<Imsi_List.size(); i++){
                    Ptr<LteUeNetDevice> UeNetDevice = m_ues[Imsi_List[i]];

                    isBestCell = UeNetDevice->GetPhy()->GetIsBestcell();

                    if(isBestCell == 1.0)
                        break;
                }
                box23->AddValue(isBestCell);
            }
        obsContainer -> Add("enbBestCell", box23);

        Ptr < OpenGymBoxContainer < double > > box24 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box24 = CreateObject < OpenGymBoxContainer < double > > (shape);
        Ptr < OpenGymBoxContainer < double > > box25 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box25 = CreateObject < OpenGymBoxContainer < double > > (shape);
        Ptr < OpenGymBoxContainer < double > > box26 = CreateObject < OpenGymBoxContainer < double > > (shape);
        box26 = CreateObject < OpenGymBoxContainer < double > > (shape);

        for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter !=m_enbs.end(); ++iter){
          uint32_t CellId = iter->first;      
          box24->AddValue(enbStepPrb[CellId]);
          box25->AddValue(eNBStepRlf[CellId]);
          box26->AddValue(eNBStepPp[CellId]);
        }
        obsContainer->Add("enbStepPrb",box24);
        obsContainer->Add("enbStepRlf",box25);
        obsContainer->Add("enbStepPp",box26);


    return obsContainer;
    }

    void
    MyGymEnv::resetObs() {
        NS_LOG_FUNCTION(this);
        m_rbUtil.assign(m_cellCount, 0);
        rewards.assign(m_cellCount, 0);
        rewards_sum.assign(m_cellCount, 0);
        m_cellFrequency.assign(m_cellCount, 0);
        m_dlThroughputVec.assign(m_cellCount, 0);
        m_dlThroughput = 0;
        UserThrouput.clear();
        m_UesNum.assign(m_cellCount, 0);
        std::vector < uint32_t > dummyVec(29, 0);
        m_MCSPen.assign(m_cellCount, dummyVec);
        NS_LOG_LOGIC("%%%%%%%% Stop collecting %%%%%%%%  time= " << Simulator::Now().GetSeconds() << " sec");
        collect = 0;
        Simulator::Schedule(Seconds(m_interval - collecting_window), & MyGymEnv::Start_Collecting, this);
    }

    void
    MyGymEnv::calculate_rewards() {
        std::map < uint16_t, std::map < uint16_t, float >> ::iterator itr;
       

        std::map < uint16_t, float > ::iterator ptr;
        int Blocked_Users_num = 0;
        double min_thro = 100;
	    m_UesNum.assign(m_cellCount, 0);
        float sum_reward=0 ;
        for (itr = UserThrouput.begin(); itr != UserThrouput.end(); itr++) {
            float all = 0;
            std::map < uint16_t, float > tempmap = itr -> second;
            m_UesNum.at(itr -> first - 1) = tempmap.size();
            
            for (ptr = itr -> second.begin(); ptr != itr -> second.end(); ptr++) {
                if (ptr -> second < block_Thr)
                    Blocked_Users_num++;
                if (ptr -> second < min_thro)
                    min_thro = ptr -> second;
                NS_LOG_LOGIC("rnti: " << ptr -> first << " throughput:  " << ptr -> second);
                all = all + ptr -> second;
            }

            NS_LOG_LOGIC("using sum Cell: " << itr -> first << " total throuput: " << all);
            rewards.at(itr -> first - 1) = all;
            sum_reward += all;
        }

        for (itr = UserThrouput.begin(); itr != UserThrouput.end(); itr++) {
            rewards_sum.at(itr->first - 1) = sum_reward;
        }
    }

    float
    MyGymEnv::GetReward() {
        NS_LOG_FUNCTION(this);

        float reward = 0;
        // reward = rewards.at(0);
        // if (m_chooseReward == 0) {
        //     reward = rewards.at(0);
        // } else if (m_chooseReward == 1) {
        //     reward = rewards.at(1);
        // } else if (m_chooseReward == 2) {
        //     reward = rewards.at(2);
        // } else {
        //     NS_LOG_ERROR("m_chooseReward variable should be between 0-2");
        // }
        resetObs();
        // NS_LOG_LOGIC("MyGetReward: " << reward);
        return reward;
    }

    std::string
    MyGymEnv::GetExtraInfo() {
        NS_LOG_FUNCTION(this);
        return "";
    }

    bool
    MyGymEnv::ExecuteActions(Ptr < OpenGymDataContainer > action) {
        
        
        // Ptr<OpenGymDictContainer> dict = DynamicCast<OpenGymDictContainer>(action);
        Ptr<OpenGymBoxContainer<float> > box = DynamicCast<OpenGymBoxContainer<float>>(action);

        // For Joint Q-learning
        ///////////////////////////////////////////
        if(m_port == 1167 || m_port == 1403){
            
            uint32_t nodeNum = m_enbs.size();
            
            // QLB Action
            /////////////////
            std::list<LteRrcSap::CellsToAddMod> celllist_temp;

            for(uint32_t i = 0; i < nodeNum; i++){
                LteRrcSap::CellsToAddMod cell_temp;
                cell_temp.cellIndex = i+1;
                cell_temp.physCellId = 0;

                int8_t cio = box->GetValue(i);

                cell_temp.cellIndividualOffset = cio;

                std::cout<<"Cell "<<i+1<<"  new CIO: "<<int(cio)<<std::endl;

                celllist_temp.push_back(cell_temp);
                
            }
            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter != m_enbs.end(); ++iter){
                iter->second->m_rrc->setCellstoAddModList(celllist_temp);
                std::map<uint16_t, Ptr<UeManager>> m_UeMap = iter->second->GetRrc()->m_ueMap;
                for(auto iter2 = m_UeMap.begin(); iter2 != m_UeMap.end(); iter2++)
                {
                    iter2->second->ScheduleRrcConnectionRecursive();
                }
            }
            /////////////////

            // QMRO Action
            /////////////////


            std::vector<double> UeActions (120);

            for (std::map<uint32_t, Ptr<LteEnbNetDevice>>::iterator iter = m_enbs.begin(); iter != m_enbs.end(); ++iter){
                    
                uint32_t cellid = (iter->first)-1;

                // Large = 9, small = 5
                double HOM;
                uint16_t TTT;

                if(m_port == 1165|| m_port == 1166 || m_port == 1167){
                    HOM = box->GetValue(9 + 1 + 2*cellid);
                    TTT = box->GetValue(9 + 2*cellid);
                }
                else if(m_port == 1402 || m_port == 1403){
                    HOM = box->GetValue(5 + 1 + 2*cellid);
                    TTT = box->GetValue(5 + 2*cellid);
                }

                std::cout<<"Cell "<<cellid<<" HOM: "<<HOM<<" TTT: "<<TTT<<std::endl;

                std::vector<uint64_t> Imsi_List;

                std::map<uint16_t, Ptr<UeManager>> UeMap = iter->second->GetRrc ()->m_ueMap;
                    
                for (std::map<uint16_t, Ptr<UeManager>>::iterator iter2 = UeMap.begin(); iter2 != UeMap.end(); ++iter2){
                    uint64_t Imsi = iter2->second->GetImsi();

                    Ptr<LteUeNetDevice> UeNetDevice = m_ues[Imsi];

                    double velocity_x = UeNetDevice->GetNode()->GetObject<MobilityModel>()->GetVelocity().x;
                    double velocity_y = UeNetDevice->GetNode()->GetObject<MobilityModel>()->GetVelocity().y;
                    double velocity_z = UeNetDevice->GetNode()->GetObject<MobilityModel>()->GetVelocity().z;

                    double velocity = sqrt( pow(velocity_x, 2) + pow(velocity_y, 2) + pow(velocity_z, 2) );


                    double mappedVelocity;
                    if(m_port == 1167 || m_port == 1403){
                        if(velocity <=20.0){
                        mappedVelocity = 1.0;
                        }
                        else if(velocity > 20.0 && velocity <= 30.0){
                            mappedVelocity = 0.75;
                        }
                        else if(velocity > 30.0 && velocity <= 50.0){
                            mappedVelocity = 0.50;
                        }
                        else{
                            mappedVelocity = 0.25;
                        }
                    }

                    uint64_t Imsi_actions = Imsi -1;
                    
                    UeActions[2*Imsi_actions] = HOM;
                    UeActions[2*Imsi_actions+1] = TTT*mappedVelocity;
                }
            }

            for (std::map<uint64_t, Ptr<LteUeNetDevice>>::iterator iter = m_ues.begin(); iter != m_ues.end(); ++iter)
            {
                uint64_t imsi = (iter->first)-1;
                double HOM = UeActions[2*imsi];
                uint16_t TTT = UeActions[2*imsi+1];


                iter->second->GetRrc()->ChangeTtt(TTT);
                iter->second->GetRrc()->ChangeHom(HOM);
            }
            /////////////////

        }
        ///////////////////////////////////////////


        return true;
    }

    uint8_t
    MyGymEnv::Convert2ITB(uint8_t mcsIdx) {
        uint8_t iTBS;
        if (mcsIdx < 10) {
            iTBS = mcsIdx;
        } else if (mcsIdx < 17) {
            iTBS = mcsIdx - 1;
        } else {
            iTBS = mcsIdx - 2;
        }

        return iTBS;
    }

    uint8_t
    MyGymEnv::GetnRB(uint8_t iTB, uint16_t tbSize) {
        uint32_t tbSizeb = uint32_t(tbSize) * uint32_t(8);
        // search in list
    uint32_t tbList[]  = {
			  16,32,56,88,120,152,176,208,224,256,288,328,344,376,392,424,456,488,504,536,568,600,616,648,680,712,744,776,776,808,840,872,904,936,968,1000,1032,1032,1064,1096,1128,1160,1192,1224,1256,1256,1288,1320,1352,1384,1416,1416,1480,1480,1544,1544,1608,1608,1608,1672,1672,1736,1736,1800,1800,1800,1864,1864,1928,1928,1992,1992,2024,2088,2088,2088,2152,2152,2216,2216,2280,2280,2280,2344,2344,2408,2408,2472,2472,2536,2536,2536,2600,2600,2664,2664,2728,2728,2728,2792,2792,2856,2856,2856,2984,2984,2984,2984,2984,3112,
		24,56,88,144,176,208,224,256,328,344,376,424,456,488,520,568,600,632,680,712,744,776,808,872,904,936,968,1000,1032,1064,1128,1160,1192,1224,1256,1288,1352,1384,1416,1416,1480,1544,1544,1608,1608,1672,1736,1736,1800,1800,1864,1864,1928,1992,1992,2024,2088,2088,2152,2152,2216,2280,2280,2344,2344,2408,2472,2472,2536,2536,2600,2600,2664,2728,2728,2792,2792,2856,2856,2856,2984,2984,2984,3112,3112,3112,3240,3240,3240,3240,3368,3368,3368,3496,3496,3496,3496,3624,3624,3624,3752,3752,3752,3752,3880,3880,3880,4008,4008,4008,
		32,72,144,176,208,256,296,328,376,424,472,520,568,616,648,696,744,776,840,872,936,968,1000,1064,1096,1160,1192,1256,1288,1320,1384,1416,1480,1544,1544,1608,1672,1672,1736,1800,1800,1864,1928,1992,2024,2088,2088,2152,2216,2216,2280,2344,2344,2408,2472,2536,2536,2600,2664,2664,2728,2792,2856,2856,2856,2984,2984,3112,3112,3112,3240,3240,3240,3368,3368,3368,3496,3496,3496,3624,3624,3624,3752,3752,3880,3880,3880,4008,4008,4008,4136,4136,4136,4264,4264,4264,4392,4392,4392,4584,4584,4584,4584,4584,4776,4776,4776,4776,4968,4968,
		40,104,176,208,256,328,392,440,504,568,616,680,744,808,872,904,968,1032,1096,1160,1224,1256,1320,1384,1416,1480,1544,1608,1672,1736,1800,1864,1928,1992,2024,2088,2152,2216,2280,2344,2408,2472,2536,2536,2600,2664,2728,2792,2856,2856,2984,2984,3112,3112,3240,3240,3368,3368,3496,3496,3624,3624,3624,3752,3752,3880,3880,4008,4008,4136,4136,4264,4264,4392,4392,4392,4584,4584,4584,4776,4776,4776,4776,4968,4968,4968,5160,5160,5160,5352,5352,5352,5352,5544,5544,5544,5736,5736,5736,5736,5992,5992,5992,5992,6200,6200,6200,6200,6456,6456,
		56,120,208,256,328,408,488,552,632,696,776,840,904,1000,1064,1128,1192,1288,1352,1416,1480,1544,1608,1736,1800,1864,1928,1992,2088,2152,2216,2280,2344,2408,2472,2600,2664,2728,2792,2856,2984,2984,3112,3112,3240,3240,3368,3496,3496,3624,3624,3752,3752,3880,4008,4008,4136,4136,4264,4264,4392,4392,4584,4584,4584,4776,4776,4968,4968,4968,5160,5160,5160,5352,5352,5544,5544,5544,5736,5736,5736,5992,5992,5992,5992,6200,6200,6200,6456,6456,6456,6456,6712,6712,6712,6968,6968,6968,6968,7224,7224,7224,7480,7480,7480,7480,7736,7736,7736,7992,
		72,144,224,328,424,504,600,680,776,872,968,1032,1128,1224,1320,1384,1480,1544,1672,1736,1864,1928,2024,2088,2216,2280,2344,2472,2536,2664,2728,2792,2856,2984,3112,3112,3240,3368,3496,3496,3624,3752,3752,3880,4008,4008,4136,4264,4392,4392,4584,4584,4776,4776,4776,4968,4968,5160,5160,5352,5352,5544,5544,5736,5736,5736,5992,5992,5992,6200,6200,6200,6456,6456,6712,6712,6712,6968,6968,6968,7224,7224,7224,7480,7480,7480,7736,7736,7736,7992,7992,7992,8248,8248,8248,8504,8504,8760,8760,8760,8760,9144,9144,9144,9144,9528,9528,9528,9528,9528,
		328,176,256,392,504,600,712,808,936,1032,1128,1224,1352,1480,1544,1672,1736,1864,1992,2088,2216,2280,2408,2472,2600,2728,2792,2984,2984,3112,3240,3368,3496,3496,3624,3752,3880,4008,4136,4136,4264,4392,4584,4584,4776,4776,4968,4968,5160,5160,5352,5352,5544,5736,5736,5992,5992,5992,6200,6200,6456,6456,6456,6712,6712,6968,6968,6968,7224,7224,7480,7480,7736,7736,7736,7992,7992,8248,8248,8248,8504,8504,8760,8760,8760,9144,9144,9144,9144,9528,9528,9528,9528,9912,9912,9912,10296,10296,10296,10296,10680,10680,10680,10680,11064,11064,11064,11448,11448,11448,
		104,224,328,472,584,712,840,968,1096,1224,1320,1480,1608,1672,1800,1928,2088,2216,2344,2472,2536,2664,2792,2984,3112,3240,3368,3368,3496,3624,3752,3880,4008,4136,4264,4392,4584,4584,4776,4968,4968,5160,5352,5352,5544,5736,5736,5992,5992,6200,6200,6456,6456,6712,6712,6712,6968,6968,7224,7224,7480,7480,7736,7736,7992,7992,8248,8248,8504,8504,8760,8760,8760,9144,9144,9144,9528,9528,9528,9912,9912,9912,10296,10296,10296,10680,10680,10680,11064,11064,11064,11448,11448,11448,11448,11832,11832,11832,12216,12216,12216,12576,12576,12576,12960,12960,12960,12960,13536,13536,
		120,256,392,536,680,808,968,1096,1256,1384,1544,1672,1800,1928,2088,2216,2344,2536,2664,2792,2984,3112,3240,3368,3496,3624,3752,3880,4008,4264,4392,4584,4584,4776,4968,4968,5160,5352,5544,5544,5736,5992,5992,6200,6200,6456,6456,6712,6968,6968,7224,7224,7480,7480,7736,7736,7992,7992,8248,8504,8504,8760,8760,9144,9144,9144,9528,9528,9528,9912,9912,9912,10296,10296,10680,10680,10680,11064,11064,11064,11448,11448,11448,11832,11832,12216,12216,12216,12576,12576,12576,12960,12960,12960,13536,13536,13536,13536,14112,14112,14112,14112,14688,14688,14688,14688,15264,15264,15264,15264,
		136,296,456,616,776,936,1096,1256,1416,1544,1736,1864,2024,2216,2344,2536,2664,2856,2984,3112,3368,3496,3624,3752,4008,4136,4264,4392,4584,4776,4968,5160,5160,5352,5544,5736,5736,5992,6200,6200,6456,6712,6712,6968,6968,7224,7480,7480,7736,7992,7992,8248,8248,8504,8760,8760,9144,9144,9144,9528,9528,9912,9912,10296,10296,10296,10680,10680,11064,11064,11064,11448,11448,11832,11832,11832,12216,12216,12576,12576,12960,12960,12960,13536,13536,13536,13536,14112,14112,14112,14112,14688,14688,14688,15264,15264,15264,15264,15840,15840,15840,16416,16416,16416,16416,16992,16992,16992,16992,17568,
		144,328,504,680,872,1032,1224,1384,1544,1736,1928,2088,2280,2472,2664,2792,2984,3112,3368,3496,3752,3880,4008,4264,4392,4584,4776,4968,5160,5352,5544,5736,5736,5992,6200,6200,6456,6712,6712,6968,7224,7480,7480,7736,7992,7992,8248,8504,8504,8760,9144,9144,9144,9528,9528,9912,9912,10296,10296,10680,10680,11064,11064,11448,11448,11448,11832,11832,12216,12216,12576,12576,12960,12960,12960,13536,13536,13536,14112,14112,14112,14688,14688,14688,14688,15264,15264,15264,15840,15840,15840,16416,16416,16416,16992,16992,16992,16992,17568,17568,17568,18336,18336,18336,18336,18336,19080,19080,19080,19080,
		176,376,584,776,1000,1192,1384,1608,1800,2024,2216,2408,2600,2792,2984,3240,3496,3624,3880,4008,4264,4392,4584,4776,4968,5352,5544,5736,5992,5992,6200,6456,6712,6968,6968,7224,7480,7736,7736,7992,8248,8504,8760,8760,9144,9144,9528,9528,9912,9912,10296,10680,10680,11064,11064,11448,11448,11832,11832,12216,12216,12576,12576,12960,12960,13536,13536,13536,14112,14112,14112,14688,14688,14688,15264,15264,15840,15840,15840,16416,16416,16416,16992,16992,16992,17568,17568,17568,18336,18336,18336,18336,19080,19080,19080,19080,19848,19848,19848,19848,20616,20616,20616,21384,21384,21384,21384,22152,22152,22152,
		208,440,680,904,1128,1352,1608,1800,2024,2280,2472,2728,2984,3240,3368,3624,3880,4136,4392,4584,4776,4968,5352,5544,5736,5992,6200,6456,6712,6712,6968,7224,7480,7736,7992,8248,8504,8760,8760,9144,9528,9528,9912,9912,10296,10680,10680,11064,11064,11448,11832,11832,12216,12216,12576,12576,12960,12960,13536,13536,14112,14112,14112,14688,14688,15264,15264,15264,15840,15840,16416,16416,16416,16992,16992,17568,17568,17568,18336,18336,18336,19080,19080,19080,19080,19848,19848,19848,20616,20616,20616,21384,21384,21384,21384,22152,22152,22152,22920,22920,22920,23688,23688,23688,23688,24496,24496,24496,24496,25456,
		224,488,744,1000,1256,1544,1800,2024,2280,2536,2856,3112,3368,3624,3880,4136,4392,4584,4968,5160,5352,5736,5992,6200,6456,6712,6968,7224,7480,7736,7992,8248,8504,8760,9144,9144,9528,9912,9912,10296,10680,10680,11064,11448,11448,11832,12216,12216,12576,12960,12960,13536,13536,14112,14112,14688,14688,14688,15264,15264,15840,15840,16416,16416,16992,16992,16992,17568,17568,18336,18336,18336,19080,19080,19080,19848,19848,19848,20616,20616,20616,21384,21384,21384,22152,22152,22152,22920,22920,22920,23688,23688,23688,24496,24496,24496,25456,25456,25456,25456,26416,26416,26416,26416,27376,27376,27376,27376,28336,28336,
		256,552,840,1128,1416,1736,1992,2280,2600,2856,3112,3496,3752,4008,4264,4584,4968,5160,5544,5736,5992,6200,6456,6968,7224,7480,7736,7992,8248,8504,8760,9144,9528,9912,9912,10296,10680,11064,11064,11448,11832,12216,12216,12576,12960,12960,13536,13536,14112,14112,14688,14688,15264,15264,15840,15840,16416,16416,16992,16992,17568,17568,18336,18336,18336,19080,19080,19848,19848,19848,20616,20616,20616,21384,21384,22152,22152,22152,22920,22920,22920,23688,23688,24496,24496,24496,25456,25456,25456,25456,26416,26416,26416,27376,27376,27376,28336,28336,28336,28336,29296,29296,29296,29296,30576,30576,30576,30576,31704,31704,
		280,600,904,1224,1544,1800,2152,2472,2728,3112,3368,3624,4008,4264,4584,4968,5160,5544,5736,6200,6456,6712,6968,7224,7736,7992,8248,8504,8760,9144,9528,9912,10296,10296,10680,11064,11448,11832,11832,12216,12576,12960,12960,13536,13536,14112,14688,14688,15264,15264,15840,15840,16416,16416,16992,16992,17568,17568,18336,18336,18336,19080,19080,19848,19848,20616,20616,20616,21384,21384,22152,22152,22152,22920,22920,23688,23688,23688,24496,24496,24496,25456,25456,25456,26416,26416,26416,27376,27376,27376,28336,28336,28336,29296,29296,29296,29296,30576,30576,30576,30576,31704,31704,31704,31704,32856,32856,32856,34008,34008,
		328,632,968,1288,1608,1928,2280,2600,2984,3240,3624,3880,4264,4584,4968,5160,5544,5992,6200,6456,6712,7224,7480,7736,7992,8504,8760,9144,9528,9912,9912,10296,10680,11064,11448,11832,12216,12216,12576,12960,13536,13536,14112,14112,14688,14688,15264,15840,15840,16416,16416,16992,16992,17568,17568,18336,18336,19080,19080,19848,19848,19848,20616,20616,21384,21384,22152,22152,22152,22920,22920,23688,23688,24496,24496,24496,25456,25456,25456,26416,26416,26416,27376,27376,27376,28336,28336,28336,29296,29296,29296,30576,30576,30576,30576,31704,31704,31704,31704,32856,32856,32856,34008,34008,34008,34008,35160,35160,35160,35160,
		336,696,1064,1416,1800,2152,2536,2856,3240,3624,4008,4392,4776,5160,5352,5736,6200,6456,6712,7224,7480,7992,8248,8760,9144,9528,9912,10296,10296,10680,11064,11448,11832,12216,12576,12960,13536,13536,14112,14688,14688,15264,15264,15840,16416,16416,16992,17568,17568,18336,18336,19080,19080,19848,19848,20616,20616,20616,21384,21384,22152,22152,22920,22920,23688,23688,24496,24496,24496,25456,25456,26416,26416,26416,27376,27376,27376,28336,28336,29296,29296,29296,30576,30576,30576,30576,31704,31704,31704,32856,32856,32856,34008,34008,34008,35160,35160,35160,35160,36696,36696,36696,36696,37888,37888,37888,39232,39232,39232,39232,
		376,776,1160,1544,1992,2344,2792,3112,3624,4008,4392,4776,5160,5544,5992,6200,6712,7224,7480,7992,8248,8760,9144,9528,9912,10296,10680,11064,11448,11832,12216,12576,12960,13536,14112,14112,14688,15264,15264,15840,16416,16416,16992,17568,17568,18336,18336,19080,19080,19848,19848,20616,21384,21384,22152,22152,22920,22920,23688,23688,24496,24496,24496,25456,25456,26416,26416,27376,27376,27376,28336,28336,29296,29296,29296,30576,30576,30576,31704,31704,31704,32856,32856,32856,34008,34008,34008,35160,35160,35160,36696,36696,36696,37888,37888,37888,37888,39232,39232,39232,40576,40576,40576,40576,42368,42368,42368,42368,43816,43816,
		408,840,1288,1736,2152,2600,2984,3496,3880,4264,4776,5160,5544,5992,6456,6968,7224,7736,8248,8504,9144,9528,9912,10296,10680,11064,11448,12216,12576,12960,13536,13536,14112,14688,15264,15264,15840,16416,16992,16992,17568,18336,18336,19080,19080,19848,20616,20616,21384,21384,22152,22152,22920,22920,23688,24496,24496,25456,25456,25456,26416,26416,27376,27376,28336,28336,29296,29296,29296,30576,30576,30576,31704,31704,32856,32856,32856,34008,34008,34008,35160,35160,35160,36696,36696,36696,37888,37888,37888,39232,39232,39232,40576,40576,40576,40576,42368,42368,42368,43816,43816,43816,43816,45352,45352,45352,46888,46888,46888,46888,
		440,904,1384,1864,2344,2792,3240,3752,4136,4584,5160,5544,5992,6456,6968,7480,7992,8248,8760,9144,9912,10296,10680,11064,11448,12216,12576,12960,13536,14112,14688,14688,15264,15840,16416,16992,16992,17568,18336,18336,19080,19848,19848,20616,20616,21384,22152,22152,22920,22920,23688,24496,24496,25456,25456,26416,26416,27376,27376,28336,28336,29296,29296,29296,30576,30576,31704,31704,31704,32856,32856,34008,34008,34008,35160,35160,35160,36696,36696,36696,37888,37888,39232,39232,39232,40576,40576,40576,42368,42368,42368,42368,43816,43816,43816,45352,45352,45352,46888,46888,46888,46888,48936,48936,48936,48936,48936,51024,51024,51024,
		488,1000,1480,1992,2472,2984,3496,4008,4584,4968,5544,5992,6456,6968,7480,7992,8504,9144,9528,9912,10680,11064,11448,12216,12576,12960,13536,14112,14688,15264,15840,15840,16416,16992,17568,18336,18336,19080,19848,19848,20616,21384,21384,22152,22920,22920,23688,24496,24496,25456,25456,26416,26416,27376,27376,28336,28336,29296,29296,30576,30576,31704,31704,31704,32856,32856,34008,34008,35160,35160,35160,36696,36696,36696,37888,37888,39232,39232,39232,40576,40576,40576,42368,42368,42368,43816,43816,43816,45352,45352,45352,46888,46888,46888,46888,48936,48936,48936,48936,51024,51024,51024,51024,52752,52752,52752,52752,55056,55056,55056,
		520,1064,1608,2152,2664,3240,3752,4264,4776,5352,5992,6456,6968,7480,7992,8504,9144,9528,10296,10680,11448,11832,12576,12960,13536,14112,14688,15264,15840,16416,16992,16992,17568,18336,19080,19080,19848,20616,21384,21384,22152,22920,22920,23688,24496,24496,25456,25456,26416,27376,27376,28336,28336,29296,29296,30576,30576,31704,31704,32856,32856,34008,34008,34008,35160,35160,36696,36696,36696,37888,37888,39232,39232,40576,40576,40576,42368,42368,42368,43816,43816,43816,45352,45352,45352,46888,46888,46888,48936,48936,48936,48936,51024,51024,51024,51024,52752,52752,52752,55056,55056,55056,55056,57336,57336,57336,57336,59256,59256,59256,
		552,1128,1736,2280,2856,3496,4008,4584,5160,5736,6200,6968,7480,7992,8504,9144,9912,10296,11064,11448,12216,12576,12960,13536,14112,14688,15264,15840,16416,16992,17568,18336,19080,19848,19848,20616,21384,22152,22152,22920,23688,24496,24496,25456,25456,26416,27376,27376,28336,28336,29296,29296,30576,30576,31704,31704,32856,32856,34008,34008,35160,35160,36696,36696,37888,37888,37888,39232,39232,40576,40576,40576,42368,42368,43816,43816,43816,45352,45352,45352,46888,46888,46888,48936,48936,48936,51024,51024,51024,51024,52752,52752,52752,55056,55056,55056,55056,57336,57336,57336,57336,59256,59256,59256,59256,61664,61664,61664,61664,63776,
		584,1192,1800,2408,2984,3624,4264,4968,5544,5992,6712,7224,7992,8504,9144,9912,10296,11064,11448,12216,12960,13536,14112,14688,15264,15840,16416,16992,17568,18336,19080,19848,19848,20616,21384,22152,22920,22920,23688,24496,25456,25456,26416,26416,27376,28336,28336,29296,29296,30576,31704,31704,32856,32856,34008,34008,35160,35160,36696,36696,36696,37888,37888,39232,39232,40576,40576,42368,42368,42368,43816,43816,45352,45352,45352,46888,46888,46888,48936,48936,48936,51024,51024,51024,52752,52752,52752,52752,55056,55056,55056,57336,57336,57336,57336,59256,59256,59256,61664,61664,61664,61664,63776,63776,63776,63776,66592,66592,66592,66592,
		616,1256,1864,2536,3112,3752,4392,5160,5736,6200,6968,7480,8248,8760,9528,10296,10680,11448,12216,12576,13536,14112,14688,15264,15840,16416,16992,17568,18336,19080,19848,20616,20616,21384,22152,22920,23688,24496,24496,25456,26416,26416,27376,28336,28336,29296,29296,30576,31704,31704,32856,32856,34008,34008,35160,35160,36696,36696,37888,37888,39232,39232,40576,40576,40576,42368,42368,43816,43816,43816,45352,45352,46888,46888,46888,48936,48936,48936,51024,51024,51024,52752,52752,52752,55056,55056,55056,55056,57336,57336,57336,59256,59256,59256,61664,61664,61664,61664,63776,63776,63776,63776,66592,66592,66592,66592,68808,68808,68808,71112,
		712,1480,2216,2984,3752,4392,5160,5992,6712,7480,8248,8760,9528,10296,11064,11832,12576,13536,14112,14688,15264,16416,16992,17568,18336,19080,19848,20616,21384,22152,22920,23688,24496,25456,25456,26416,27376,28336,29296,29296,30576,30576,31704,32856,32856,34008,35160,35160,36696,36696,37888,37888,39232,40576,40576,40576,42368,42368,43816,43816,45352,45352,46888,46888,48936,48936,48936,51024,51024,52752,52752,52752,55056,55056,55056,55056,57336,57336,57336,59256,59256,59256,61664,61664,61664,63776,63776,63776,66592,66592,66592,68808,68808,68808,71112,71112,71112,73712,73712,75376,75376,75376,75376,75376,75376,75376,75376,75376,75376,75376};

        uint16_t count = iTB * 110;
        while (tbList[count++] != tbSizeb);

        return (count % 110);

    }

    void
    MyGymEnv::GetPhyStats(Ptr < MyGymEnv > gymEnv,
        const PhyTransmissionStatParameters params) {
        if (gymEnv -> collect == 1) {
            // Get size of TB
            uint32_t idx = params.m_cellId - 1;
            gymEnv -> m_cellFrequency.at(idx) = gymEnv -> m_cellFrequency.at(idx) + 1;
            gymEnv -> m_dlThroughputVec.at(idx) = gymEnv -> m_dlThroughputVec.at(idx) + (params.m_size) * 8.0 / 1024.0 / 1024.0 / gymEnv -> collecting_window;
            gymEnv -> m_dlThroughput = gymEnv -> m_dlThroughput + (params.m_size) * 8.0 / 1024.0 / 1024.0 / gymEnv -> collecting_window;
           
            //add throughput per user
            gymEnv -> UserThrouput[params.m_cellId][params.m_rnti] = gymEnv -> UserThrouput[params.m_cellId][params.m_rnti] + (params.m_size) * 8.0 / 1024.0 / 1024.0 / gymEnv -> collecting_window;
            // Get nRBs
            uint8_t nRBs = MyGymEnv::GetnRB(MyGymEnv::Convert2ITB(params.m_mcs), params.m_size);
            gymEnv -> m_rbUtil.at(idx) = gymEnv -> m_rbUtil.at(idx) + nRBs;
           

            // Get MCSPen
            gymEnv -> m_MCSPen.at(idx).at(params.m_mcs) = gymEnv -> m_MCSPen.at(idx).at(params.m_mcs) + 1;

        }
    }

} // ns3 namespace