Trajectory Optimization
=======================
This directory contains the C++ code responsible for:
1. Generating references trajectories for the feedback control system to track using a trajectory optimization approach

Overview
--------
1. We have a dynamical model of the delta robot.
2. We design a feedback control system, which, given measurements of the delta robot's state, computes control inputs
to track a reference trajectory subject to model uncertainty, sensor noise, and exogenous disturbances.
3. We put a box around that closed-loop system and come up with some continuous dynamics of that new system.
4. We find the set off open-loop controls (i.e. reference signals) that minimize some cost function (energy expenditure)
or some STL robustness measure that get the robot from point A to point B for that closed-loop system.
5. The feedback contorl system tracks those references while ensuring that we don't deviate from the trajectory for 
robustness.

Trajectory generation is (3) and (4) in that list. We need to come up with the closed-loop dynamics, which we pretty
much get for free. Then we discretize those dynamics (transcription). We find the control inputs (references) that 
minimize some cost function subject to the constraints of the system that bring it from an initial point to a final
point. We fit a continuous spline through those points (potentially take derivatives thereof) and pass those things
as references onto the feedback control system.
