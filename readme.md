# Lab Exercise for "Control and Perception in Networked and Autonomous Vehicles"
This repository provides a starting point for the lab exercises of the course "Control and Perception in Networked and Autonomous Vehicles". You can find the [course material on the CPM Lab website](https://cpm.embedded.rwth-aachen.de/course-materials/).

The code is organized as MATLAB packages in respective folders. 

The folder `+cmmn` provides functionality which is shared between exercises.

Run the lab exercises in the [CPM Lab](https://github.com/embedded-software-laboratory/cpm_lab).
The folder in which this code resides is expected to be `software/high_level_controller/`.

# Networked Modell Predictive Control (MPC) Strategies
The main task of this lab exercise is to implement different networked MPC strategies for networked and autonomous vehicles, especially the centralized MPC (CMPC) and distributed MPC (DMPC). Specifically, we will use priority based non-cooperative DMPC [1] for this lab exercise. The performance of these two kinds of networked MPC will be compared. The networked vehicles form a platoon, from which the leading vehicle should follow a reference speed and the following vehicles should maintain a constant distance to it's front vehicle while avoiding collision. For control purpose, linear discrete state-space model will be used by MPC to predict vehicles' behavior; thus, the non-linear behivior of the vehicle will be approvimated using linear state-space model. For model identification, the step response of the vehicle will be experimentally collected and analyzed using MATLAB's System Identification Toolbox (`ssest`). The identified system parameters are included in the system matrix A, input matrix B, output matrix C. This linear state-space modell will be used throughout the total lab exercise. The implemented algorithm will be tested using simulation and experiment. The experiment will be carried out in [Cyber-Physical Mobility Lab](https://github.com/embedded-software-laboratory/cpm_lab) at the [Chair of Embedded Software](http://embedded.rwth-aachen.de/) at [RWTH Aachen University](http://rwth-aachen.de/), where 20 networked model-scale (1:18) vehicles with maximum speed of 3.7 m/s can be used.

## I. Centralized Platoon Control
CMPC will be used in centralized platoon control. CMPC is characterized by the existence of a center controller that solves the optimization problem for the whole platoon and then send the optimized control input to each vehicle. The coupling and communication graph are shown in Fig. 1.

In this lab exercise, there are five vehicles and they form a platoon. Then, we find a suitable state-space representation of the networked control system (NCS). Specifically, the NCS will be represented by a whole system matrix A_total, input matrix B_total and output matrix C_total. The output of the system is a five-dimentional vector. The first element is the velocity of the leading vehicle, other four elements are the distance between the adjacent vehicles. Thus, the optimization problem of the CMPC is to find a optimal input trajectory, which minimize the leading vehicle's velocity error and the adjacent vehicles' distance error under consideration of volocity and acceleration constraints. For a collision free control, the minimal distance between each vehicle should be larger than 0.3 m. The optimization problem will be formed as quadratic objective function with linear constraints such that it can be solved using MATLAB toolbox `quadprog`. The results are shown in Fig. 2 and Fig. 3.

## II. Distributed Platoon Control
DMPC will be used in distributed platoon control. There are two kinds of DMPC, cooperative DMPC and non-cooperative MPC. The most difference between them is that the former consider other vehicles' objective function while the latter not. In this lab exercise, non-cooperative DMPC will be used. To deal with the prediction inconsistence problem [1, p. 49] of the non-cooperative DMPC, in [1, p. 51] a novel non-cooperative DMPC is proposed, where each vehicle will be prioritized. The priority-based non-cooperative DMPC can reduce conputation time as well. In our simple scerario, priority can be easily assigned: leading vehicle has highest priority, second vehicle has the second highest priority, and so on. The optimization problem is solved sequentially (see Tab. 1). In the `for-loop`, the leading vehicle's optimal input trajectory will be firstly calculated; then, the predicted position under using the optimal control input trajectory will be send to the second vehile to calculate it's reference position for the optimization problem. This will be repeated until every vehicle's optimal control input is calculated. Finally, the optimal control input will be apply to each vehicle. Though in each step five optimization problems are solved, the total computation time is lower than CMPC because the dimention of each optimization problem in DMPC is far less than that in CMPC, which makes the algorithm easily to find the optimal solution. 

Tab. 1: Pseudo code of sequential computation
```
u_total = zeros(5,1) # initialize the control input for each vehicle

for i=1:5 # for the case of five vehicles
  if i == 1 # leading vehicle
    [u_1,y_1] = optimizeMPC(v_ref) # optimization problem with reference velocity trajectory as input
    u_total(1) = u_1(1)
  else # other vehicles
    [u_i,y_i] = optimizeMPC(y_(i-1)) # optimization problem with position of the front vehicle as input
    u_total(i) = u_i(1)
  end
end

apply(u_1,u_2,u_3,u_4,u_5) # apply control input to each vehicle
```
The results are shown in Fig. 4 and Fig. 5.

# Reference
