# RL-Clock-Drift-Correction

Clock synchronization in the Internet of Things (IoT) is a critical aspect of ensuring reliable and energy-efficient communications among devices within a network. In this paper, we propose an entirely autonomous and lightweight Reinforcement Learning (RL) approach to learn the periodicity of synchronized beacon transmissions between a transmitter and several receivers, while maximizing the sleep time between successive beacons to conserve energy. To do so, the proposed approach exploits a set of states, actions, and rewards so that each device adapts the radio-on time accordingly. The approach runs on each individual receiver without any prior knowledge of the network status. It is implemented and tested on off-the-shelf ESP32 IoT devices which are known to exhibit high clock drift rates. The testbed results demonstrate the ability of the approach to autonomously synchronize the receivers while achieving a similar performance in terms of packet (beacon) reception ratio but 45\% better energy efficiency compared to a traditional approach followed in the literature for one-to-many type of synchronization. Apart from the improved energy consumption, the power characterization of the system shows that the RL approach requires negligible CPU resources.

## Gateway
Contains the source code for the gateway. 

## resp_without_ds_full_rl_sync
Contains the RL solution for receiver devices
