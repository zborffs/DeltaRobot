function [configuration, i, delta_xs]= forward_kinematics(phi, initial_guess, p)
% forward_kinematics numerically computes the configuration of the robot 
% from measurements of the actuated joint angles.

% extract state variables from phi
q11 = phi(1);
q21 = phi(2);
q31 = phi(3);

% declare the convergence tolerances
x_abstol = 1e-32;
max_iter = 100;
delta_x = inf;

% extract the robot parameters
l1 = p.l1;
l2 = p.l2;
r_base = p.r_base;
r_platform = p.r_platform;

% declare loop variables
i = 0;
delta_xs = [];
x0 = initial_guess;

while (delta_x == inf || abs(delta_x) > x_abstol) && i < max_iter
    % rest of the configuration we are trying to guess
    q12 = x0(1);
    q13 = x0(2);
    q22 = x0(3);
    q23 = x0(4);
    q32 = x0(5);
    q33 = x0(6);
    
    J = [l2*cos(q12)*sin(q13), l2*cos(q13)*sin(q12), (l2*(cos(q22)*sin(q23) + 3^(1/2)*sin(q22)))/2, (l2*cos(q23)*sin(q22))/2, 0, 0; -l2*sin(q12), 0, -(l2*(sin(q22) - 3^(1/2)*cos(q22)*sin(q23)))/2, (3^(1/2)*l2*cos(q23)*sin(q22))/2, 0, 0; l2*cos(q12)*cos(q13), -l2*sin(q12)*sin(q13), -l2*cos(q22)*cos(q23), l2*sin(q22)*sin(q23), 0, 0; l2*cos(q12)*sin(q13), l2*cos(q13)*sin(q12), 0, 0, (l2*(cos(q32)*sin(q33) - 3^(1/2)*sin(q32)))/2, (l2*cos(q33)*sin(q32))/2; -l2*sin(q12), 0, 0, 0, -(l2*(sin(q32) + 3^(1/2)*cos(q32)*sin(q33)))/2, -(3^(1/2)*l2*cos(q33)*sin(q32))/2; l2*cos(q12)*cos(q13), -l2*sin(q12)*sin(q13), 0, 0, -l2*cos(q32)*cos(q33), l2*sin(q32)*sin(q33)];
    h = [l2*sin(q12)*sin(q13) - (3^(1/2)*(r_base - r_platform + l1*cos(q21) + l2*cos(q22)))/2 + (l2*sin(q22)*sin(q23))/2; (3*r_base)/2 - (3*r_platform)/2 + l1*cos(q11) + l2*cos(q12) + (l1*cos(q21))/2 + (l2*cos(q22))/2 + (3^(1/2)*l2*sin(q22)*sin(q23))/2; l1*sin(q11) - l1*sin(q21) + l2*cos(q13)*sin(q12) - l2*cos(q23)*sin(q22); (3^(1/2)*(r_base - r_platform + l1*cos(q31) + l2*cos(q32)))/2 + l2*sin(q12)*sin(q13) + (l2*sin(q32)*sin(q33))/2; (3*r_base)/2 - (3*r_platform)/2 + l1*cos(q11) + l2*cos(q12) + (l1*cos(q31))/2 + (l2*cos(q32))/2 - (3^(1/2)*l2*sin(q32)*sin(q33))/2; l1*sin(q11) - l1*sin(q31) + l2*cos(q13)*sin(q12) - l2*cos(q33)*sin(q32)];
%     
%     if sum(J.^2) < 1e-12
%         configuration = [q11; x0(1); x0(2); q21; x0(3); x0(4); q31; x0(5); x0(6)];
%         return;
%     end
    
    xk = x0 - J \ h;
    delta_x = max(abs(xk - x0)); % okay
    
    x0 = xk;
    i = i + 1;
    
    delta_xs(i) = delta_x;
end

configuration = wrapToPi([q11; x0(1); x0(2); q21; x0(3); x0(4); q31; x0(5); x0(6)]);

end