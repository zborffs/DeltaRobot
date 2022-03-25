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

%% Run the dynamics generation script to put matrices in MATLAB workspace
addpath("utils");
gen_delta_dynamics_lagrangian; % careful! running this will nuke workspace

%% Setup 'B' matrix
B = zeros(9,3); B(1,1) = 1; B(4,2) = 1; B(7,3) = 1;

%% Compute the Projection matrix P
% P = eye(9,9) - H' * inv(H * invM * H') * H * invM; % -> not needed yet

%% Take this (https://www.intechopen.com/chapters/52037) approach

% Rearrange the state variables so the actuated are the top 3
qtilde = [q11; q21; q31; q12; q13; q22; q23; q32; q33];
qdottilde = [q11dot; q21dot; q31dot; q12dot; q13dot; q22dot; q23dot; q32dot; q33dot];
qddottilde = [q11ddot; q21ddot; q31ddot; q12ddot; q13ddot; q22ddot; q23ddot; q32ddot; q33ddot];

% explicitly label these state vectors 'u' and 'a' for actuated and unact.
qa = qtilde(1:3);
qu = qtilde(4:9);
qadot = qdottilde(1:3);
qudot = qdottilde(4:9);

% Rearrange 'M' matrix to correspond with new state vector
Mtilde = [M(1,:); M(4,:); M(7,:); M(2:3,:); M(5:6,:); M(8:9,:)];
Mtilde = [Mtilde(:,1) Mtilde(:,4) Mtilde(:,7) Mtilde(:,2:3) Mtilde(:,5:6) Mtilde(:,8:9)];
M11 = Mtilde(1:3,1:3);
M12 = Mtilde(1:3,4:9);
M21 = Mtilde(4:9,1:3);
M22 = Mtilde(4:9,4:9);

% Rearrange 'C' matrix to correspond with new state vector
Ctilde = [C(1); C(4); C(7); C(2:3); C(5:6); C(8:9)];

% Recompute christoffel symbols (out of interest for it's cool properties)
Gammatilde = christoffel_symbols(Mtilde, qtilde, length(qtilde)); % compute the Christoffel symbols
for i = 1:size(Gammatilde, 1)
    assert(simplify(Ctilde(i) - qdottilde' * reshape(Gammatilde(i, :, :), size(qdottilde, 1), size(qdottilde, 1)) * qdottilde) == 0)
end

% super weird property, but there is no coupling between unactuated and
% actuated christoffel symbols
Gammatilde_1_1 = reshape(Gammatilde(1, :, :), size(qdottilde, 1), size(qdottilde, 1));
Gammatilde_1_2 = reshape(Gammatilde(2, :, :), size(qdottilde, 1), size(qdottilde, 1));
Gammatilde_1_3 = reshape(Gammatilde(3, :, :), size(qdottilde, 1), size(qdottilde, 1));

Gammatilde_2_1 = reshape(Gammatilde(4, :, :), size(qdottilde, 1), size(qdottilde, 1));
Gammatilde_2_2 = reshape(Gammatilde(5, :, :), size(qdottilde, 1), size(qdottilde, 1));
Gammatilde_2_3 = reshape(Gammatilde(6, :, :), size(qdottilde, 1), size(qdottilde, 1));
Gammatilde_2_4 = reshape(Gammatilde(7, :, :), size(qdottilde, 1), size(qdottilde, 1));
Gammatilde_2_5 = reshape(Gammatilde(8, :, :), size(qdottilde, 1), size(qdottilde, 1));
Gammatilde_2_6 = reshape(Gammatilde(9, :, :), size(qdottilde, 1), size(qdottilde, 1));

% Rearrange gravity matrix to be consistent with state vectors
Gtilde = [G(1); G(4); G(7); G(2:3); G(5:6); G(8:9)];
G1 = Gtilde(1:3);
G2 = Gtilde(4:9);
C1 = Ctilde(1:3);
C2 = Ctilde(4:9);

% Solve for 'quddot' in 'internal' dynamics (i.e. unactuated dynamics)
invM22 = inv(M22);  % 3x3 diagonal matrix... very easy to invert...

% Substitute this into actuated dynamics
t1 = simplify(-M12 * invM22);
Mbar = simplify(M11 + t1 * M21);
Cbar = simplify(t1 * C2 + C1);
Gbar = simplify(t1 * G2 + G1);

% Mbar * qaddot + Cbar + Gbar = u  % depends on all 'q's and 'qdot's

% necessary to stabilize that, but not sufficient. It is also necessary to 
% stabilize the unactuated dynamics from control inputs. Solve for 'qaddot'
% as a function of full state and control. Substitute into 'quddot'
% equation. Is that equation stable?



