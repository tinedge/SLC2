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

#ifndef MY_GYM_ENTITY_H

# define MY_GYM_ENTITY_H

#include "ns3/stats-module.h"

#include "ns3/opengym-module.h"

#include <vector>

#include <ns3/lte-common.h>

#include <map>

#include "ns3/lte-enb-net-device.h"

#include "ns3/lte-enb-rrc.h"

#include "ns3/lte-ue-net-device.h"
#include "ns3/lte-ue-rrc.h"
#include "ns3/radio-bearer-stats-calculator.h"

#include "ns3/nstime.h"
#include "ns3/ff-mac-scheduler.h"
namespace ns3 {

    class MyGymEnv: public OpenGymEnv {
        public: 
            MyGymEnv();
            MyGymEnv(double stepTime, uint32_t N1, uint32_t N2, uint16_t N3, uint32_t port);
            virtual~MyGymEnv();
            static TypeId GetTypeId(void);
            virtual void DoDispose();
            Ptr < OpenGymSpace > GetActionSpace();
            Ptr < OpenGymSpace > GetObservationSpace();
            bool GetGameOver();
            Ptr < OpenGymDataContainer > GetObservation();
            float GetReward();
            void calculate_rewards();
            std::string GetExtraInfo();
            bool ExecuteActions(Ptr < OpenGymDataContainer > action);
            static void GetPhyStats(Ptr < MyGymEnv > gymEnv,
                const PhyTransmissionStatParameters params);
            static void SetInitNumofUEs(Ptr < MyGymEnv > gymEnv, std::vector < uint32_t > num);
            static void GetNumofUEs(Ptr < MyGymEnv > gymEnv, uint16_t CellDec, uint16_t CellInc, uint16_t CellInc_Ues);
            void AddNewNode(uint16_t cellId, Ptr<LteEnbNetDevice> dev);
            void AddNewUe(uint64_t imsi, Ptr<LteUeNetDevice> dev);
            
            void GetRlcStats(Ptr<RadioBearerStatsCalculator> m_rlcStats); // NS-3 SON

            private: void ScheduleNextStateRead();
            void Start_Collecting();
            static uint8_t Convert2ITB(uint8_t MCSidx);
            static uint8_t GetnRB(uint8_t iTB, uint16_t tbSize);
            void resetObs();
            uint32_t collect;
            float m_step;
            double collecting_window = 0.01;
            double block_Thr = 0.5;
            uint32_t m_cellCount;
            uint32_t m_userCount;
            uint32_t m_port;
            uint32_t m_nRBTotal;
            uint8_t m_chooseReward;
            int RLF_Counter = 0 ; //kihoon 0523
            int Pingpong_Counter = 0; //kihoon 0523
            int Step_Counter = 0; //kihoon 0523
            std::vector < uint32_t > m_cellFrequency;
            std::vector < uint32_t > m_UesNum;
            std::vector < float > m_dlThroughputVec;
            float m_dlThroughput;
            std::vector < std::vector < uint32_t >> m_MCSPen;
            std::vector < uint32_t > m_rbUtil;
            std::vector < float > rewards;
            std::vector < float > rewards_sum;
            double m_interval = 0.1;
            std::map < uint16_t,
            std::map < uint16_t,
            float > > UserThrouput;
            
            std::map<uint32_t, Ptr<LteEnbNetDevice> > m_enbs;
            std::map<uint64_t, Ptr<LteUeNetDevice>> m_ues;

            std::map<uint64_t, uint32_t> dlThroughput_IMSI; // NS-3 SON
            std::map<uint64_t, uint32_t> ulThroughput_IMSI; // NS-3 SON
            Ptr<RadioBearerStatsCalculator> RlcStats; // NS-3 SON
            std::vector <float> CurrentCIO; // NS-3 SON

            // std::vector <int> RlfCount;
            // std::map<uint64_t, int> Imsi_RlfCount;


            int step = 0;
            std::map<uint32_t, float> preenbMLBstate; 
            std::map<uint32_t, double> preenbneigborMLBindicator;

    };

}

#endif // MY_GYM_ENTITY_H
