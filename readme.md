# Lab Exercise for "Control and Perception in Networked and Autonomous Vehicles"
This repository provides a starting point for the lab exercises of the course "Control and Perception in Networked and Autonomous Vehicles". You can find the [course material on the CPM Lab website](https://cpm.embedded.rwth-aachen.de/course-materials/).

The code is organized as MATLAB packages in respective folders. 

The folder `+cmmn` provides functionality which is shared between exercises.

Run the lab exercises in the [CPM Lab](https://github.com/embedded-software-laboratory/cpm_lab).
The folder in which this code resides is expected to be `software/high_level_controller/`.

# Networked Modell Predictive Control (MPC) Strategies
The main task of this lab exercise is to implement different networked MPC strategies for networked and autonomous vehicles, especially the centralized MPC (CMPC) and distributed MPC (DMPC). Specifically, we will use priority based non-cooperative DMPC (PB-Non-Coop. DMPC) [1] for this lab exercise. The performance of these two kinds of networked MPC will be compared. The networked vehicles form a platoon, from which the leading vehicle should follow a reference speed and the following vehicles should maintain a constant distance to it's front vehicle while avoiding collision. For control purpose, linear discrete state-space model will be used by MPC to predict vehicles' behavior; thus, the non-linear behivior of the vehicle will be approvimated using linear state-space model. For model identification, the step response of the vehicle will be experimentally collected and analyzed using MATLAB's System Identification Toolbox (`ssest`). The identified system parameters are included in the system matrix A, input matrix B, output matrix C. This linear state-space modell will be used throughout the total lab exercise. The implemented algorithm will be tested using simulation and experiment. The experiment will be carried out in Cyber-Physical Mobility Lab at the Chair of Embedded Software at RWTH Aachen University.

## I. Centralized Platoon Control
CMPC will be used to control centralized platoon. CMPC is characterized by the existence of a center controller that solves the optimization problem for the whole platoon and then send the optimized control input to each vehicle (see Fig. 1).

In this lab exercise, there are five vehicles and they form a platoon. Then, we find a suitable state-space representation of the networked control system (NCS). Specifically, the NCS will be represented by a whole system matrix A_total, input matrix B_total and output matrix C_total. The output of the system is a five-dimentional vector. The first element is the velocity of the leading vehicle, other four elements are the distance between the adjacent vehicles. Thus, the optimization problem of the CMPC is to find a optimal input trajectory, which minimize the leading vehicle's velocity error and the following vehicles' relative distance error under consideration of volocity and acceleration constraints. For safety reason, the minimal distance between each vehicle should be bigger than 0.3 m.

## II. Distributed Platoon Control


# Reference
