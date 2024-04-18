import random
import gym
import numpy as np
import math 
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import collections
from ns3gym import ns3env
import matplotlib.pyplot as plt

import csv

def action_func_Mro(actions):
    env_actions = []

    Hom = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    Ttt = [100, 128, 256, 320, 480, 512, 640]
    
    Hom_len = len(Hom)
    for i in actions:
        env_action = divmod(i, Hom_len)
        Hom_action = Hom[env_action[0]]
        Ttt_action = Ttt[env_action[1]]
        
        env_actions.append(Ttt_action)
        env_actions.append(Hom_action)

    return env_actions

def action_func_Mlb(actions):
    env_actions = []

    CIO = [-2, -1, 0, 1, 2,]

    for i in range(len(actions)):
        CIO_action = CIO[actions[i]]

        env_actions.append(CIO_action)
        
    return env_actions

def action_func_coordinator(actions):
    env_actions_coordinator = []
    actions = actions[0]

    for i in range(9) :
        env_actions_coordinator.append(actions % 4) 
        actions = int(actions/4)
  
        
    return env_actions_coordinator

def csv2list(file_path):
    result = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            result.append(row)
    return result

device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

EPISODES = 2000 
max_env_steps = 30 
port=1167
stepTime=0.5
startSim=0
seed=3
simArgs = {}
debug=True


######################################################################################################################################################
# Sum Tree Class #####################################################################################################################################
######################################################################################################################################################
class SumTree:
    def __init__(self, size):
        self.nodes = [0] * (2 * size - 1)
        self.data = [None] * size

        self.size = size
        self.count = 0
        self.real_size = 0

    @property
    def total(self):
        return self.nodes[0]

    def update(self, data_idx, value):
        idx = data_idx + self.size - 1  # child index in tree array
        change = value - self.nodes[idx]

        self.nodes[idx] = value

        parent = (idx - 1) // 2
        while parent >= 0:
            self.nodes[parent] += change
            parent = (parent - 1) // 2

    def add(self, value, data):
        self.data[self.count] = data
        self.update(self.count, value)

        self.count = (self.count + 1) % self.size
        self.real_size = min(self.size, self.real_size + 1)

    def get(self, cumsum):
        assert cumsum <= self.total

        idx = 0
        while 2 * idx + 1 < len(self.nodes):
            left, right = 2*idx + 1, 2*idx + 2

            if cumsum <= self.nodes[left]:
                idx = left
            else:
                idx = right
                cumsum = cumsum - self.nodes[left]

        data_idx = idx - self.size + 1

        return data_idx, self.nodes[idx], self.data[data_idx]

    def __repr__(self):
        return f"SumTree(nodes={self.nodes.__repr__()}, data={self.data.__repr__()})"
######################################################################################################################################################
    

######################################################################################################################################################
# Prioritize Experience Replay Class ################################################################################################################
######################################################################################################################################################
class PrioritizedReplayBuffer:
    def __init__(self, state_size, action_size, buffer_size, eps=1e-2, alpha=0.1, beta=0.1):
        self.tree = SumTree(size=buffer_size)

        # PER params
        self.eps = eps  # minimal priority, prevents zero probabilities
        self.alpha = alpha  # determines how much prioritization is used, α = 0 corresponding to the uniform case
        self.beta = beta  # determines the amount of importance-sampling correction, b = 1 fully compensate for the non-uniform probabilities
        self.max_priority = eps  # priority for new samples, init as eps

        # transition: state, action, reward, next_state, done
        self.state = torch.empty(buffer_size, state_size, dtype=torch.float)
        # self.action = torch.empty(buffer_size, action_size, dtype=torch.float)
        self.action = torch.empty(buffer_size, dtype=torch.int64)
        self.reward = torch.empty(buffer_size, dtype=torch.float)
        self.next_state = torch.empty(buffer_size, state_size, dtype=torch.float)
        
        self.count = 0
        self.real_size = 0
        self.size = buffer_size

    def add(self, transition):
        state, action, reward, next_state = transition

        # store transition index with maximum priority in sum tree
        self.tree.add(self.max_priority, self.count)

        self.state[self.count] = torch.as_tensor(state)
        self.action[self.count] = torch.as_tensor(action)
        self.reward[self.count] = torch.as_tensor(reward)
        self.next_state[self.count] = torch.as_tensor(next_state)

        # update counters
        self.count = (self.count + 1) % self.size
        self.real_size = min(self.size, self.real_size + 1)

    def sample(self, batch_size):
        assert self.real_size >= batch_size, "buffer contains less samples than batch size"

        sample_idxs, tree_idxs = [], []
        priorities = torch.empty(batch_size, 1, dtype=torch.float)

        segment = self.tree.total / batch_size
        for i in range(batch_size):
            a, b = segment * i, segment * (i + 1)

            cumsum = random.uniform(a, b)

            tree_idx, priority, sample_idx = self.tree.get(cumsum)

            priorities[i] = priority
            tree_idxs.append(tree_idx)
            sample_idxs.append(sample_idx)

        probs = priorities / self.tree.total
        weights = (self.real_size * probs) ** -self.beta
        weights = weights / weights.max()

        batch = (
            self.state[sample_idxs].to(device),
            self.action[sample_idxs].to(device).unsqueeze(1),
            self.reward[sample_idxs].to(device).unsqueeze(1),
            self.next_state[sample_idxs].to(device),
        )
        return batch, weights, tree_idxs

    def update_priorities(self, data_idxs, priorities):
        if isinstance(priorities, torch.Tensor):
            priorities = priorities.detach().cpu().numpy()

        for data_idx, priority in zip(data_idxs, priorities.squeeze()):

            priority = (priority + self.eps) ** self.alpha

            self.tree.update(data_idx, priority)
            self.max_priority = max(self.max_priority, priority)
######################################################################################################################################################

######################################################################################################################################################
# DDQN Class #########################################################################################################################################
######################################################################################################################################################
class DDQNAgent:
    def __init__(self, state_size, action_size):
        
        self.state_size = state_size
        self.action_size = action_size
        self.capacity = 20000

        self.memory = PrioritizedReplayBuffer(self.state_size, self.action_size, self.capacity)

        self.gamma = 0.95  
        self.epsilon = 1.0
        self.epsilon_end = 0.01
        self.epsilon_decay = 0.999
        self.steps_done = 0
        self.learning_rate = 0.001
        self.train_start = 500 
        self.model = self._build_model()
        self.target_model = self._build_model()

        self.optimizer = optim.Adam(self.model.parameters(), self.learning_rate)

        self.update_target_model()

        self.loss = 0

    def _build_model(self):
        model = nn.Sequential(
            nn.Linear(self.state_size, 32),
            nn.ReLU(),
            nn.Linear(32,32),
            nn.ReLU(),
            nn.Linear(32,self.action_size)
        )
        return model.to(device)
    
    def update_target_model(self):
        self.target_model.load_state_dict(self.model.state_dict())

    def act(self, state):
        self.steps_done += 1
        
        act_values = self.model(torch.tensor(state, dtype=torch.float32, device=device))

        if np.random.rand() <= self.epsilon:
            print("Random Action")
            return torch.LongTensor([random.randrange(self.action_size)]).to(device) 
        else :
            print("Agent Action")
            return torch.argmax(act_values[0]).unsqueeze(0).to(device) 
    
    def remember(self, state, action, reward, next_state):

        self.memory.add((state, action, reward, next_state))
        
        if (self.epsilon > self.epsilon_end) :
            self.epsilon *= self.epsilon_decay
        print("Epsilon: ",self.epsilon)
    
    def learn(self, batch_size):
        
        if self.steps_done < self.train_start:
            print("Saved Data Num: ",self.steps_done)
            print("Not learning")
            return
        
        print("Saved Data Num: ",self.steps_done)
        print("Learning")

        batch, weights, tree_idxs = self.memory.sample(batch_size)

        weights = weights.to(device)

        print('tree indexes in learn function: ',tree_idxs)

        states, actions, rewards, next_states  = batch

        current_q = self.model(states).gather(1,actions)

        max_action = torch.argmax(self.model(next_states),dim=1).unsqueeze(0)
        max_actions = max_action.transpose(0,1)
        max_next_q = self.target_model(next_states).gather(1,max_actions)
        
        expected_q = rewards + (self.gamma * max_next_q)

        td_errors = torch.abs(current_q - expected_q)

        self.memory.update_priorities(tree_idxs, td_errors.cpu().detach().numpy())

        loss = (weights * F.mse_loss(current_q, expected_q)).mean()

        self.loss = loss
        print("Model Loss: ",float(loss.item()))

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
######################################################################################################################################################

# Main Code
if __name__ == "__main__" :

    env = ns3env.Ns3Env(port=port, stepTime=stepTime, startSim=startSim, simSeed=seed, simArgs=simArgs, debug=debug)

    ObsSpace_Mro = 11
    ActSpace_Mro = 49

    ObsSpace_Mlb = 36
    ActSpace_Mlb = 5

    n_agents = 1

    state_size = 54
    action_size = 4**9

    done = False
    batch_size = 64 

    lossList = []

    
    break_ep = 0
    FullStep_Ep = []

    previous_CIO = [0] * 9 
    previous_TTT = [0] * 9 
    previous_HOM = [0] * 9
    actions_Mro_previous = [0] * 9
    actions_Mlb_previous = [0] * 9

    Q1_Mlb = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable1_QLB_large.csv"))
    Q2_Mlb = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable2_QLB_large.csv"))
    Q3_Mlb = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable3_QLB_large.csv"))
    Q4_Mlb = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable4_QLB_large.csv"))
    Q5_Mlb = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable5_QLB_large.csv"))
    Q6_Mlb = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable6_QLB_large.csv"))
    Q7_Mlb = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable7_QLB_large.csv"))
    Q8_Mlb = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable8_QLB_large.csv"))
    Q9_Mlb = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable9_QLB_large.csv"))

    Q1_Mro = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable1_QMRO_large.csv"))
    Q2_Mro = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable2_QMRO_large.csv"))
    Q3_Mro = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable3_QMRO_large.csv"))
    Q4_Mro = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable4_QMRO_large.csv"))
    Q5_Mro = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable5_QMRO_large.csv"))
    Q6_Mro = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable6_QMRO_large.csv"))
    Q7_Mro = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable7_QMRO_large.csv"))
    Q8_Mro = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable8_QMRO_large.csv"))
    Q9_Mro = np.array(csv2list("/home/mnc2/Eunsok/ETRI/Qtable/large/Qtable9_QMRO_large.csv"))

    Coordinator = DDQNAgent(state_size, action_size)

    torch.autograd.set_detect_anomaly(True)

    for ep in range(EPISODES):

        # Reset environment and get first new observation
        state = env.reset()
        done = False
        print("ep(EPISODE) : ",ep)

        # QMRO
        #############################
        state_Mro = np.reshape(state['AverageVelocity'], [9,1])

        state1_Mro = int(state_Mro[0])
        state2_Mro = int(state_Mro[1])
        state3_Mro = int(state_Mro[2])
        state4_Mro = int(state_Mro[3])
        state5_Mro = int(state_Mro[4])
        state6_Mro = int(state_Mro[5])
        state7_Mro = int(state_Mro[6])
        state8_Mro = int(state_Mro[7])
        state9_Mro = int(state_Mro[8])
        #############################

        # QLB
        #############################
        state_Mlb = np.reshape(state['enbMLBstate'], [9,1])
        state1_Mlb = int(state_Mlb[0])
        state2_Mlb = int(state_Mlb[1])
        state3_Mlb = int(state_Mlb[2])
        state4_Mlb = int(state_Mlb[3])
        state5_Mlb = int(state_Mlb[4])
        state6_Mlb = int(state_Mlb[5])
        state7_Mlb = int(state_Mlb[6])
        state8_Mlb = int(state_Mlb[7])
        state9_Mlb = int(state_Mlb[8])
        #############################
        
        # The Q-Table learning algorithm
        for j in range(max_env_steps): 
            print("*******************************")
            print("Episode: %d"%(ep+1))
            print("Step: %d"%(j+1))

            # QMRO
            if j == 0: ############################################
                action1_Mro = np.argmax(Q1_Mro[state1_Mro, :].astype(float))
                action2_Mro = np.argmax(Q2_Mro[state2_Mro, :].astype(float))
                action3_Mro = np.argmax(Q3_Mro[state3_Mro, :].astype(float))
                action4_Mro = np.argmax(Q4_Mro[state4_Mro, :].astype(float))
                action5_Mro = np.argmax(Q5_Mro[state5_Mro, :].astype(float))
                action6_Mro = np.argmax(Q6_Mro[state6_Mro, :].astype(float))
                action7_Mro = np.argmax(Q7_Mro[state7_Mro, :].astype(float))
                action8_Mro = np.argmax(Q8_Mro[state8_Mro, :].astype(float))
                action9_Mro = np.argmax(Q9_Mro[state9_Mro, :].astype(float))

                actions_Mro = []
                actions_Mro.append(action1_Mro)
                actions_Mro.append(action2_Mro)
                actions_Mro.append(action3_Mro)
                actions_Mro.append(action4_Mro)
                actions_Mro.append(action5_Mro)
                actions_Mro.append(action6_Mro)
                actions_Mro.append(action7_Mro)
                actions_Mro.append(action8_Mro)
                actions_Mro.append(action9_Mro)

                env_actions_Mro = action_func_Mro(actions_Mro)
                ############################################

                # QLB
                ############################################
                action1_Mlb = np.argmax(Q1_Mlb[state1_Mlb, :].astype(float))
                action2_Mlb = np.argmax(Q2_Mlb[state2_Mlb, :].astype(float))
                action3_Mlb = np.argmax(Q3_Mlb[state3_Mlb, :].astype(float))
                action4_Mlb = np.argmax(Q4_Mlb[state4_Mlb, :].astype(float))
                action5_Mlb = np.argmax(Q5_Mlb[state5_Mlb, :].astype(float))
                action6_Mlb = np.argmax(Q6_Mlb[state6_Mlb, :].astype(float))
                action7_Mlb = np.argmax(Q7_Mlb[state7_Mlb, :].astype(float))
                action8_Mlb = np.argmax(Q8_Mlb[state8_Mlb, :].astype(float))
                action9_Mlb = np.argmax(Q9_Mlb[state9_Mlb, :].astype(float))
                
                actions_Mlb = []
                actions_Mlb.append(action1_Mlb)
                actions_Mlb.append(action2_Mlb)
                actions_Mlb.append(action3_Mlb)
                actions_Mlb.append(action4_Mlb)
                actions_Mlb.append(action5_Mlb)
                actions_Mlb.append(action6_Mlb)
                actions_Mlb.append(action7_Mlb)
                actions_Mlb.append(action8_Mlb)
                actions_Mlb.append(action9_Mlb)

                env_actions_Mlb = action_func_Mlb(actions_Mlb)
                ############################################
                

                for k in range(9):
                    print("ENB {} actions HOM: {}  TTT: {}".format(k+1, env_actions_Mro[2*k+1], env_actions_Mro[2*k]))
                    print("ENB {} actions CIO: {} ".format(k+1, env_actions_Mlb[k]))
                
                env_actions = []
                for n in env_actions_Mlb:
                    env_actions.append(n)

                for m in env_actions_Mro:
                    env_actions.append(m)  
                
                CurrentCio = np.array(env_actions[:9]) #CIO
                CurrentHom = np.array(env_actions[9::2][:9]) #HOM
                CurrentTtt = np.array(env_actions[10::2][:9]) #TTT

                PreviousCio = np.array(previous_CIO) # previous_CIO
                PreviousHom = np.array(previous_HOM) # previous_HOM        
                PreviousTtt = np.array(previous_TTT) # previous_TTT

                cioAction = []
                homAction = []
                tttAction = []

                for i in range(9):
                    if CurrentCio[i] > PreviousCio[i]:
                        cioAction.append(1.0)
                    elif CurrentCio[i] == PreviousCio[i]:
                        cioAction.append(0.0)
                    else:
                        cioAction.append(-1.0)
                    
                    if CurrentHom[i] > PreviousHom[i]:
                        homAction.append(1.0)
                    elif CurrentHom[i] == PreviousHom[i]:
                        homAction.append(0.0)
                    else:
                        homAction.append(-1.0)

                    if CurrentTtt[i] > PreviousTtt[i]:
                        tttAction.append(1.0)
                    elif CurrentTtt[i] == PreviousTtt[i]:
                        tttAction.append(0.0)
                    else:
                        tttAction.append(-1.0)
                
                state1_coordinator = np.array(cioAction)
                state1_coordinator = np.reshape(state1_coordinator, (9, 1))
                state1_coordinator = state1_coordinator.astype(np.float64)

                state2_coordinator = np.array(homAction)
                state2_coordinator = np.reshape(state2_coordinator, (9, 1))
                state2_coordinator = state2_coordinator.astype(np.float64)

                state3_coordinator = np.array(tttAction)
                state3_coordinator = np.reshape(state3_coordinator, (9, 1))
                state3_coordinator = state3_coordinator.astype(np.float64)

                state4_coordinator  = np.reshape(state['AvgCqi'], [9,1])
                state4_coordinator = state4_coordinator.astype(np.float64)

                state5_coordinator = np.reshape(state['dlPrbusage'], [9,1])
                state5_coordinator = state5_coordinator.astype(np.float64)

                state6_coordinator = np.reshape(state['enbBestCell'], [9,1])
                state6_coordinator = state6_coordinator.astype(np.float64)

                Coordinator_state = np.concatenate( (state1_coordinator, 
                                                     state2_coordinator, 
                                                     state3_coordinator, 
                                                     state4_coordinator,
                                                     state5_coordinator,
                                                     state6_coordinator) )
                Coordinator_state = np.reshape(Coordinator_state, [1,state_size])

                coordinator_state_array = np.array(Coordinator_state)
                Coordinator_state = coordinator_state_array

            env_index = []
            action_coordinator = Coordinator.act(Coordinator_state)

            actions_coordinator = action_coordinator.unsqueeze(1).cpu().numpy()
            env_index.append(action_coordinator.item())  
            
            print("env_index",env_index)

            env_actions_coordinator = action_func_coordinator(env_index)

            env_actions_chose = [0] * 27

            for i in range(9) :
                if (env_actions_coordinator[i] == 0) :
                    env_actions_chose[i] = env_actions[i]
                    env_actions_chose[2*i+9] = env_actions[2*i+9]
                    env_actions_chose[2*i+10] = env_actions[2*i+10]
                    
                elif (env_actions_coordinator[i] == 1) :
                    env_actions_chose[i] = env_actions[i]
                    env_actions_chose[2*i+9] = previous_HOM[i]
                    env_actions_chose[2*i+10] = previous_TTT[i]
                    
                elif (env_actions_coordinator[i] == 2) :
                    env_actions_chose[i] = previous_CIO[i]
                    env_actions_chose[2*i+9] = env_actions[2*i+9]
                    env_actions_chose[2*i+10] = env_actions[2*i+10]

                elif (env_actions_coordinator[i] == 3) :
                    env_actions_chose[i] = previous_CIO[i]
                    env_actions_chose[2*i+9] = previous_HOM[i]
                    env_actions_chose[2*i+10] = previous_TTT[i]


            for k in range(9) :
                print("ENB {} actions HOM: {}  TTT: {}".format(k+1, env_actions_chose[2*k+10], env_actions_chose[2*k+9]))
                print("ENB {} actions CIO: {} ".format(k+1, env_actions_chose[k]))
                
                if (env_actions_coordinator[k] == 0) :
                    print(" MRO : O MLB : O ")
                elif (env_actions_coordinator[k] == 1) :    
                    print(" MRO : X  MLB : O ")
                elif (env_actions_coordinator[k] == 2) :    
                    print(" MRO : O MLB : X  ")
                elif (env_actions_coordinator[k] == 3) :
                    print(" MRO : X MLB : X  ")
    
            
            # Get new state and reward from environment
            new_state, reward, done, _ = env.step(env_actions_chose)

            if new_state is None:
                if j != 27 :
                    break_ep = break_ep +1
                else:
                    FullStep_Ep.append(ep+1)
                
                break
            
            print("break_ep: ",break_ep)
            print("Full Step Episode: ",FullStep_Ep)

            Results = new_state['Results']
            print("Total RLF: ",Results[0])
            print("Total PP: ", Results[1])

            # QMRO
            ######################################
            new_state_Mro = np.reshape(new_state['AverageVelocity'], [9,1])

            new_state1_Mro = int(new_state_Mro[0])
            new_state2_Mro = int(new_state_Mro[1])
            new_state3_Mro = int(new_state_Mro[2])
            new_state4_Mro = int(new_state_Mro[3])
            new_state5_Mro = int(new_state_Mro[4])
            new_state6_Mro = int(new_state_Mro[5])
            new_state7_Mro = int(new_state_Mro[6])
            new_state8_Mro = int(new_state_Mro[7])
            new_state9_Mro = int(new_state_Mro[8])

            # QMRO choose action
            action_Mro_coordinator = [0] * 9

            for i in range(9):
                if (env_actions_coordinator[i] == 0 or env_actions_coordinator[i] == 2):
                    action_Mro_coordinator[i] = actions_Mro[i]
                elif (env_actions_coordinator[i] == 1 or env_actions_coordinator[i] == 3):
                    action_Mro_coordinator[i] = actions_Mro_previous[i]         

            
            state1_Mro = new_state1_Mro
            state2_Mro = new_state2_Mro
            state3_Mro = new_state3_Mro
            state4_Mro = new_state4_Mro
            state5_Mro = new_state5_Mro
            state6_Mro = new_state6_Mro
            state7_Mro = new_state7_Mro
            state8_Mro = new_state8_Mro
            state9_Mro = new_state9_Mro
            ######################################

            # QLB
            ######################################
            new_state_Mlb = np.reshape(new_state['enbMLBstate'], [9,1])

            new_state1_Mlb = int(new_state_Mlb[0])
            new_state2_Mlb = int(new_state_Mlb[1])
            new_state3_Mlb = int(new_state_Mlb[2])
            new_state4_Mlb = int(new_state_Mlb[3])
            new_state5_Mlb = int(new_state_Mlb[4])
            new_state6_Mlb = int(new_state_Mlb[5])
            new_state7_Mlb = int(new_state_Mlb[6])
            new_state8_Mlb = int(new_state_Mlb[7])
            new_state9_Mlb = int(new_state_Mlb[8])

            action_Mlb_coordinator = [0] * 9
            for i in range(9):
                if (env_actions_coordinator[i] == 0 or env_actions_coordinator[i] == 1):
                    action_Mlb_coordinator[i] = actions_Mlb[i]   
                elif (env_actions_coordinator[i] == 2 or env_actions_coordinator[i] == 3):
                    action_Mlb_coordinator[i] = actions_Mlb_previous[i]           

            state1_Mlb = new_state1_Mlb
            state2_Mlb = new_state2_Mlb
            state3_Mlb = new_state3_Mlb
            state4_Mlb = new_state4_Mlb
            state5_Mlb = new_state5_Mlb
            state6_Mlb = new_state6_Mlb
            state7_Mlb = new_state7_Mlb
            state8_Mlb = new_state8_Mlb
            state9_Mlb = new_state9_Mlb
  
            new_action1_MRO = np.argmax(Q1_Mro[state1_Mro, :].astype(float))
            new_action2_MRO = np.argmax(Q1_Mro[state2_Mro, :].astype(float))
            new_action3_MRO = np.argmax(Q1_Mro[state3_Mro, :].astype(float))
            new_action4_MRO = np.argmax(Q1_Mro[state4_Mro, :].astype(float))
            new_action5_MRO = np.argmax(Q1_Mro[state5_Mro, :].astype(float))
            new_action6_MRO = np.argmax(Q1_Mro[state6_Mro, :].astype(float))
            new_action7_MRO = np.argmax(Q1_Mro[state7_Mro, :].astype(float))
            new_action8_MRO = np.argmax(Q1_Mro[state8_Mro, :].astype(float))
            new_action9_MRO = np.argmax(Q1_Mro[state9_Mro, :].astype(float))

            new_actions_Mro = []
            new_actions_Mro.append(new_action1_MRO)
            new_actions_Mro.append(new_action2_MRO)
            new_actions_Mro.append(new_action3_MRO)
            new_actions_Mro.append(new_action4_MRO)
            new_actions_Mro.append(new_action5_MRO)
            new_actions_Mro.append(new_action6_MRO)
            new_actions_Mro.append(new_action7_MRO)
            new_actions_Mro.append(new_action8_MRO)
            new_actions_Mro.append(new_action9_MRO)

            new_env_actions_Mro = action_func_Mro(new_actions_Mro)
            
            actions_Mro = new_actions_Mro
            env_actions_Mro = new_env_actions_Mro

            new_action1_Mlb = np.argmax(Q1_Mlb[state1_Mlb, :].astype(float))
            new_action2_Mlb = np.argmax(Q2_Mlb[state2_Mlb, :].astype(float))
            new_action3_Mlb = np.argmax(Q3_Mlb[state3_Mlb, :].astype(float))
            new_action4_Mlb = np.argmax(Q4_Mlb[state4_Mlb, :].astype(float))
            new_action5_Mlb = np.argmax(Q5_Mlb[state5_Mlb, :].astype(float))
            new_action6_Mlb = np.argmax(Q6_Mlb[state6_Mlb, :].astype(float))
            new_action7_Mlb = np.argmax(Q7_Mlb[state7_Mlb, :].astype(float))
            new_action8_Mlb = np.argmax(Q8_Mlb[state8_Mlb, :].astype(float))
            new_action9_Mlb = np.argmax(Q9_Mlb[state9_Mlb, :].astype(float))


            new_actions_Mlb = []
            new_actions_Mlb.append(new_action1_Mlb)
            new_actions_Mlb.append(new_action2_Mlb)
            new_actions_Mlb.append(new_action3_Mlb)
            new_actions_Mlb.append(new_action4_Mlb)
            new_actions_Mlb.append(new_action5_Mlb)
            new_actions_Mlb.append(new_action6_Mlb)
            new_actions_Mlb.append(new_action7_Mlb)
            new_actions_Mlb.append(new_action8_Mlb)
            new_actions_Mlb.append(new_action9_Mlb)

            new_env_actions_Mlb = action_func_Mlb(new_actions_Mlb)
            env_actions_Mlb = new_env_actions_Mlb

            actions_Mlb = new_actions_Mlb

            ###################### coordinator new state  #################
            CurrentCio = np.array(new_env_actions_Mlb[:9]) #CIO            
            CurrentHom = np.array(new_env_actions_Mro[0::2][:9]) #HOM
            CurrentTtt = np.array(new_env_actions_Mro[1::2][:9]) #TTT

            PreviousCio = np.array(env_actions[:9]) # previous_CIO
            PreviousHom = np.array(env_actions[9::2][:9]) # previous_HOM        
            PreviousTtt = np.array(env_actions[10::2][:9]) # previous_TTT

            cioAction = []
            homAction = []
            tttAction = []

            print("Current CIO: ",CurrentCio)
            print("Current HOM: ",CurrentHom)
            print("Current TTT: ",CurrentTtt)

            print("Previous CIO: ",PreviousCio)
            print("Previous HOM: ",PreviousHom)
            print("Previous TTT: ",PreviousTtt)


            for t in range(9):
                if CurrentCio[t] > PreviousCio[t]:
                    cioAction.append(1.0)
                elif CurrentCio[t] == PreviousCio[t]:
                    cioAction.append(0.0)
                else:
                    cioAction.append(-1.0)
                    
                if CurrentHom[t] > PreviousHom[t]:
                    homAction.append(1.0)
                elif CurrentHom[t] == PreviousHom[t]:
                    homAction.append(0.0)
                else:
                    homAction.append(-1.0)

                if CurrentTtt[t] > PreviousTtt[t]:
                    tttAction.append(1.0)
                elif CurrentTtt[t] == PreviousTtt[t]:
                    tttAction.append(0.0)
                else:
                    tttAction.append(-1.0)
                
            new_state1_coordinator = np.array(cioAction)
            new_state1_coordinator = np.reshape(state1_coordinator, (9, 1))
            new_state1_coordinator = state1_coordinator.astype(np.float64)

            new_state2_coordinator = np.array(homAction)
            new_state2_coordinator = np.reshape(state2_coordinator, (9, 1))
            new_state2_coordinator = state2_coordinator.astype(np.float64)

            new_state3_coordinator = np.array(tttAction)
            new_state3_coordinator = np.reshape(state3_coordinator, (9, 1))
            new_state3_coordinator = state3_coordinator.astype(np.float64)

            new_state4_coordinator  = np.reshape(new_state['AvgCqi'], [9,1])
            new_state4_coordinator = new_state4_coordinator.astype(np.float64)

            new_state5_coordinator = np.reshape(new_state['dlPrbusage'], [9,1])
            new_state5_coordinator = new_state5_coordinator.astype(np.float64)

            new_state6_coordinator = np.reshape(new_state['enbBestCell'], [9,1])
            new_state6_coordinator = new_state6_coordinator.astype(np.float64)

            new_Coordinator_state = np.concatenate( (new_state1_coordinator, 
                                                     new_state2_coordinator, 
                                                     new_state3_coordinator, 
                                                     new_state4_coordinator,
                                                     new_state5_coordinator,
                                                     new_state6_coordinator) )
            
            new_Coordinator_state = np.reshape(new_Coordinator_state, [1,state_size])

            new_coordinator_state_array = np.array(new_Coordinator_state)
            new_Coordinator_state = new_coordinator_state_array

            enbStepPrb = np.reshape(new_state['enbStepPrb'],[9,1]).squeeze()
            enbStepRlf = np.reshape(new_state['enbStepRlf'],[9,1]).squeeze()
            enbStepPp = np.reshape(new_state['enbStepPp'],[9,1]).squeeze()

            load_std = np.std(enbStepPrb/100)
            rlf_rate = (np.sum(enbStepRlf) / 60)
            pp_rate = (np.sum(enbStepPp) / 60)

            hoap = rlf_rate * 0.7 + pp_rate * 0.3

            load_std_min = 0.008
            load_std_max = 0.314

            hoap_min = 0
            hoap_max = 0.7

            normalized_load_std = (load_std - load_std_min) / (load_std_max - load_std_min)
            normalized_hoap = (hoap - hoap_min) / (hoap_max - hoap_min)

            reward_weight = 2.75
            coordi_reward = (-normalized_load_std - reward_weight*normalized_hoap)*50
            
            coordi_reward = np.reshape(coordi_reward,[1,1])
            coordi_reward = coordi_reward[0:1,0:1]
            coordi_reward = np.round(coordi_reward,5)
            print("Coordinator Reward: ",coordi_reward)

            Coordinator.remember(Coordinator_state, actions_coordinator, coordi_reward, new_Coordinator_state)

            Coordinator_state = new_Coordinator_state 

            Coordinator.learn(batch_size)

            if((j%13) == 0) :
                print("Target network update")
                Coordinator.update_target_model()

            env_actions = []
            for n in new_env_actions_Mlb:
                env_actions.append(n)

            for m in new_env_actions_Mro:
                env_actions.append(m)    
                
            for k in range(9):
                previous_CIO[k] = env_actions_chose[k]
                previous_HOM[k] = env_actions_chose[2*k+9]
                previous_TTT[k] = env_actions_chose[2*k+10]
                actions_Mro_previous[k] = actions_Mro[k]
                actions_Mlb_previous[k] = actions_Mlb[k]
             