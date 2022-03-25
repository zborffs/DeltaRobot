%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:                                                            %
%                                                                         %
%      This script is responsible for simulating the open-loop delta      %
%      robot dynamics using Lagrangian mechanics.                         %
%                                                                         %
%      Author: Zach Bortoff                                               %
%      Last Updated: March 25, 2022                                       %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

addpath('utils')

%% Start by defining the delta robot parameters
% delta_robot_params = struct('m1', 2.5, 'm2', 3.0, 'm3', 4.0, 'l1', 0.35, 'l2', 0.50, 'g', 9.8, 'r_base', 0.125, 'r_platform', 0.03, 'd1', 0.5, 'd2', 0.5, 'd3', 0.5); % lots of damping
delta_robot_params = struct('m1', 2.5, 'm2', 3.0, 'm3', 4.0, 'l1', 0.35, 'l2', 0.50, 'g', 9.8, 'r_base', 0.125, 'r_platform', 0.03, 'd1', 0.0, 'd2', 0.0, 'd3', 0.0); % no damping
% delta_robot_params = struct('m1', 2.5, 'm2', 3.0, 'm3', 4.0, 'l1', 0.382, 'l2', 0.673, 'g', 9.8, 'r_base', 0.110, 'r_platform', 0.03, 'd1', 0.1, 'd2', 0.2, 'd3', 0.1);

%% Define the initial conditions of the problem
x0 = [pi/4, 3*pi/4, 0, pi/4, 3*pi/4, 0, pi/4, 3*pi/4, 0, zeros(1,9), zeros(1,6)];  % make the holonomic constraint is satisfies here
tspan = [0 30];

%% Define the ODE solver tolerances
M = diag([ones(18,1); zeros(6,1)]);
% options = odeset('Mass', M, 'RelTol', 1e-4, 'AbsTol', 1e-3 * ones(24, 1));
options = odeset('Mass', M, 'RelTol', 1e-6, 'AbsTol', 1e-4 * ones(24, 1));

%% Solve the IC Problem numerically using ODE23t or ODE 15s (DAE solvers)
[t, y] = ode23t(@(t,y)delta_OL_L(t, y, delta_robot_params), tspan, x0, options);
% [t, y] = ode15s(@(t,y)delta_OL_L(t, y, delta_robot_params), tspan, x0, options);

%% Compute Lagrangian at every point to measure accuracy of simulation
L = lagrangian(t, y, delta_robot_params);
plot(t, L, 'Color', 'k', 'MarkerSize', 0.5); title("Lagrangian over Time (in Simulation)"); xlabel("Time (s)"); ylabel("Lagrangian (J)"); hold on;
plot(t, L(1) * ones(size(L)), 'Color', 'g', 'MarkerSize', 1.0); hold off;

%% Create an animation of the robot over time
h = animate(t, y, delta_robot_params);
