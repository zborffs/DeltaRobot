Robot Dynamics
==============

This directory contains the code responsible for: 
1. Generating the dynamics of the delta robot symbolically both using the Lagrangian and Hamiltonian formulations,
2. Generating the dynamics of the gripper symbolically using the Newtonian formulation,
3. Generating the expressions necessary for computing the forward kinematics numerically, 
4. Performing both open- and closed-loop simulations of the delta robot,
5. Performing both open- and closed-loop simulations of the gripper,and 
6. Visualizing the robot's workspace.

Generating Dynamics using Lagrangian Dynamics
---------------------------------------------

For a more comprehensive assessment of exactly how I go about symbolically generating the dynamics of the robot using 
Lagrangian mechanics, check out the documentation or go through the code yourself.

Ultimately, I end up with a system of 15 non-linear differential-algebraic equations (DAEs), which can be solved 
numerically using an index-1 DAE solver after applying Baumegarte index reduction.

Below is a simulation of the open-loop system without damping.

Figure 1.

The simulation does not account for collisions between the links. Clearly, small numerically imprecisions get amplified
over the course of the simulation as evidenced by the end-effector deviating from the vertical bobbing that one should 
expect to be preserved in a perfect simulation. Despite this, the simulation is quite good at broadly capturing the 
movement of the robot, and can be used for coming up with and assessing the quality of feedback control laws.

Below is a simulation of the open-loop system with damping.

Figure 2.

The robot in this simulation has the exact same model parameters and initial conditions as the previous simulation, but
the underlying model is different. In this simulation I introduce some light Rayleigh dissipation akin to viscous 
damping occurring due to friction at the joints. Over time, the system is brought to rest.

The following figure depicts the energy of the overall system of the first simulation. 

Figure 3.

Despite there not being any dissipative or external forces acting the system, the energy of the system increases over 
the course of the simulation. This is a common side-effect of performing numerical simulations like these.

The graph below provides some summary statistics about the computational performance of these simulations as one 
increases the simulation time.  

Figure 4.

Clearly the performance ..., . 


Generating Dynamics using Hamiltonian Dynamics
----------------------------------------------

