
SLC2 README
================================

## OS and software preparation:

We base our experiment environment on Ubuntu 20.04 LTS and highly recommend that you do the same. This streamlines the setup process and avoids unexpected issues cause by incompatible software versions etc. Please make sure that you have Python installed. Also make sure that you have root or sudo permission.

This branch contains the entire ns-3 network simulator (ns-3.33) with ns3-gym (opengym) module.

## Install ns-3 and dependencies

1. The first part of the preparation is to clone the repository:

```shell
git clone https://github.com/tinedge/SLC2.git
```

2. Next, install all dependencies required by ns-3.

```shell
apt-get install gcc g++ python python3-pip
```

3. Install ZMQ and Protocol Buffers libs:

```shell
sudo apt-get update
apt-get install libzmq5 libzmq3-dev
apt-get install libprotobuf-dev
apt-get install protobuf-compiler
```

4. Install Pytorch.

```shell
pip3 install torch
```

Following guideline of installation in https://pytorch.org

5. Building ns-3

```shell
chmod +x ./waf
./waf configure
./waf build
```

6. Install ns3-gym

```shell
pip3 install --user ./src/opengym/model/ns3gym
```

## Running ns-3 environment

To run small scale scenario, open the terminal and run the command:

```shell
chmod +x ./SLC2_small.sh
./bash SLC2_small.sh
```

Note that, you don't have to repeat the following command after your first running.

```shell
chmod +x ./SLC2_small.sh
```

If you want to run only one episode, run the command:

```shell
./waf --run scratch/NS3_Env_small.cc
```

To run large scale scenario, open the terminal and run the command:

```shell
chmod +x ./SLC2_large.sh
./bash SLC2_large.sh
```

Note that, you don't have to repeat the following command after your first running.

```shell
chmod +x ./SLC2_large.sh
```

If you want to run only one episode, run the command:

```shell
./waf --run scratch/NS3_Env_large.cc
```

## Running agent

In the directory scratch, there are SLC2 agent files for small and large scale scenarios.

For small scale scenario, open a new terminal and run the command:

```shell
cd ./scratch
python3 SLC2_Agent_small.py
```

For large scale scenario, open a new terminal and run the command:

```shell
cd ./scratch
python3 SLC2_Agent_large.py
```

Contact
================================
Eunsok Lee, Korea University, tinedge@korea.ac.kr

Kihoon Kim, Korea University, rlgns1109@korea.ac.kr

Subin Han, Korea University, subin993@korea.ac.kr

How to reference SLC2?
================================
Please use the following bibtex:

Eunsok Lee, Kihoon Kim, Subin Han, and Sangheon Pack, "A Scalable and Low-Complexity Coordination Framework for Self-Organizing Networks," in *Proc. IEEE Vehicular Technology Conference (VTC) 2024 Fall*, Washington DC, USA, October 2024.
