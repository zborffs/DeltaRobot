%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:                                                            %
%                                                                         %
%      This script is responsible for generating several feedback control %
%      laws for the delta robot.                                          %
%                                                                         %
%      Author: Zach Bortoff                                               %
%      Last Updated: March 23, 2022                                       %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Run the dynamics generation script to put constraint in workspace
addpath("utils");
gen_delta_dynamics_lagrangian; % careful! running this will nuke workspace

%% Setup 'B' matrix
B = zeros(9,3); B(1,1) = 1; B(4,2) = 1; B(7,3) = 1;

%% Compute the Projection matrix P
% P = eye(9,9) - H' * inv(H * invM * H') * H * invM; % -> not needed yet

%% Take this (https://www.intechopen.com/chapters/52037) approach
Mtilde = [M(1,:); M(4,:); M(7,:); M(2:3,:); M(5:6,:); M(8:9,:)];
Mtilde = [Mtilde(:,1) Mtilde(:,4) Mtilde(:,7) Mtilde(:,2:3) Mtilde(:,5:6) Mtilde(:,8:9)];
M11 = Mtilde(1:3,1:3);
M12 = Mtilde(1:3,4:9);
M21 = Mtilde(4:9,1:3);
M22 = Mtilde(4:9,4:9);

Ctilde = [C(1); C(4); C(7); C(2:3); C(5:6); C(8:9)];