%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:                                                            %
%                                                                         %
%      This script is responsible for generating the delta robot's        %
%      forward kinematics symbolically.                                   %
%                                                                         %
%      Author: Zach Bortoff                                               %
%      Last Updated: March 23, 2022                                       %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run the dynamics generation script to put constraint in workspace
addpath("utils");
gen_delta_dynamics_lagrangian; % careful! running this will nuke workspace

%% Compute Jacobian of holonomic constraint w.r.t. un-actuated joints
J = simplify(simplify(jacobian(h, [q12, q13, q22, q23, q32, q33])));

%% Numerically compute "q_{k+1} = q_k - J^{-1} * h" at runtime