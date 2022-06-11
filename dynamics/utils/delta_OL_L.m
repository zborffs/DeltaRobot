function out = delta_OL_L(t, x, p)
% delta_OL_L(t, x, p) is the DAE solver update function for the dynamics of
% the delta robot in open-loop as described using Lagrangian mechanics

    %% extract the individual state variables out of the 'x' argument
    q11 = x(1);
    q12 = x(2);
    q13 = x(3);
    q21 = x(4);
    q22 = x(5);
    q23 = x(6);
    q31 = x(7);
    q32 = x(8);
    q33 = x(9);
    q11dot = x(10);
    q12dot = x(11);
    q13dot = x(12);
    q21dot = x(13);
    q22dot = x(14);
    q23dot = x(15);
    q31dot = x(16);
    q32dot = x(17);
    q33dot = x(18);
    
    %% extract the six lagrange multipliers from the 'x' argument
    lambda1 = x(19);
    lambda2 = x(20);
    lambda3 = x(21);
    lambda4 = x(22);
    lambda5 = x(23);
    lambda6 = x(24);
   
   
    %% Extract the delta robot parameters from the 'p' argument
    m1 = p.m1;
    m2 = p.m2;
    m3 = p.m3;
    l1 = p.l1;
    l2 = p.l2;
    g = p.g;
    r_base = p.r_base;
    r_platform = p.r_platform;
    d1 = p.d1;
    d2 = p.d2;
    d3 = p.d3;
    
    %% Construct the 'M' matrix out of parameters and state variables
    Mtop = [(l1^2*(m1 + 3*m2 + m3))/3, (l1*l2*(sin(q11)*sin(q12) + cos(q11)*cos(q12)*cos(q13))*(3*m2 + 2*m3))/6, -(l1*l2*cos(q11)*sin(q12)*sin(q13)*(3*m2 + 2*m3))/6; (l1*l2*(sin(q11)*sin(q12) + cos(q11)*cos(q12)*cos(q13))*(3*m2 + 2*m3))/6, (l2^2*(m2 + m3))/3, 0; -(l1*l2*cos(q11)*sin(q12)*sin(q13)*(3*m2 + 2*m3))/6, 0, -(l2^2*(m2 + m3)*(cos(q12)^2 - 1))/3];
    Mmid = [(l1^2*(m1 + 3*m2 + m3))/3, (l1*l2*(sin(q21)*sin(q22) + cos(q21)*cos(q22)*cos(q23))*(3*m2 + 2*m3))/6, -(l1*l2*cos(q21)*sin(q22)*sin(q23)*(3*m2 + 2*m3))/6; (l1*l2*(sin(q21)*sin(q22) + cos(q21)*cos(q22)*cos(q23))*(3*m2 + 2*m3))/6, (l2^2*(m2 + m3))/3, 0; -(l1*l2*cos(q21)*sin(q22)*sin(q23)*(3*m2 + 2*m3))/6, 0, -(l2^2*(m2 + m3)*(cos(q22)^2 - 1))/3];
    Mbot = [(l1^2*(m1 + 3*m2 + m3))/3, (l1*l2*(sin(q31)*sin(q32) + cos(q31)*cos(q32)*cos(q33))*(3*m2 + 2*m3))/6, -(l1*l2*cos(q31)*sin(q32)*sin(q33)*(3*m2 + 2*m3))/6; (l1*l2*(sin(q31)*sin(q32) + cos(q31)*cos(q32)*cos(q33))*(3*m2 + 2*m3))/6, (l2^2*(m2 + m3))/3, 0; -(l1*l2*cos(q31)*sin(q32)*sin(q33)*(3*m2 + 2*m3))/6, 0, -(l2^2*(m2 + m3)*(cos(q32)^2 - 1))/3];
    
    %% Take the inverse of the 'M' matrix
    invMtop = inv(Mtop); % computes the inverse of the top-left 3x3
    invMmid = inv(Mmid); % computes the inverse of the middle-center 3x3
    invMbot = inv(Mbot);% computes the inverse of the bottom-right 3x3
    invM1 = [invMtop(1,:) 0 0 0 0 0 0]; 
    invM2 = [invMtop(2,:) 0 0 0 0 0 0];
    invM3 = [invMtop(3,:) 0 0 0 0 0 0];
    invM4 = [0 0 0 invMmid(1,:) 0 0 0];
    invM5 = [0 0 0 invMmid(2,:) 0 0 0];
    invM6 = [0 0 0 invMmid(3,:) 0 0 0];
    invM7 = [0 0 0 0 0 0 invMbot(1,:)];
    invM8 = [0 0 0 0 0 0 invMbot(2,:)];
    invM9 = [0 0 0 0 0 0 invMbot(3,:)];
    
    % reconstructs the inverse from the sub-matrix inverses
    invM = [invM1; invM2; invM3; invM4; invM5; invM6; invM7; invM8; invM9];
    
    %% Computes 'b', which is b = -C - G + H' * lambda
    b = [ (l1*(6*lambda3*cos(q11) + 6*lambda6*cos(q11) - 6*lambda2*sin(q11) - 6*lambda5*sin(q11) + 3*g*m1*cos(q11) + 6*g*m2*cos(q11) + 2*g*m3*cos(q11) - 3*l2*m2*q12dot^2*cos(q12)*sin(q11) - 2*l2*m3*q12dot^2*cos(q12)*sin(q11) + 3*l2*m2*q12dot^2*cos(q11)*cos(q13)*sin(q12) + 3*l2*m2*q13dot^2*cos(q11)*cos(q13)*sin(q12) + 2*l2*m3*q12dot^2*cos(q11)*cos(q13)*sin(q12) + 2*l2*m3*q13dot^2*cos(q11)*cos(q13)*sin(q12) + 6*l2*m2*q12dot*q13dot*cos(q11)*cos(q12)*sin(q13) + 4*l2*m3*q12dot*q13dot*cos(q11)*cos(q12)*sin(q13)))/6; (l2*(6*lambda3*cos(q12)*cos(q13) - 6*lambda5*sin(q12) - 6*lambda2*sin(q12) + 6*lambda6*cos(q12)*cos(q13) + 6*lambda1*cos(q12)*sin(q13) + 6*lambda4*cos(q12)*sin(q13) + l2*m2*q13dot^2*sin(2*q12) + l2*m3*q13dot^2*sin(2*q12) + 3*g*m2*cos(q12)*cos(q13) + 2*g*m3*cos(q12)*cos(q13) - 3*l1*m2*q11dot^2*cos(q11)*sin(q12) - 2*l1*m3*q11dot^2*cos(q11)*sin(q12) + 3*l1*m2*q11dot^2*cos(q12)*cos(q13)*sin(q11) + 2*l1*m3*q11dot^2*cos(q12)*cos(q13)*sin(q11)))/6; -(l2*(6*lambda3*sin(q12)*sin(q13) - 6*lambda4*cos(q13)*sin(q12) - 6*lambda1*cos(q13)*sin(q12) + 6*lambda6*sin(q12)*sin(q13) + 3*g*m2*sin(q12)*sin(q13) + 2*g*m3*sin(q12)*sin(q13) + 2*l2*m2*q12dot*q13dot*sin(2*q12) + 2*l2*m3*q12dot*q13dot*sin(2*q12) + 3*l1*m2*q11dot^2*sin(q11)*sin(q12)*sin(q13) + 2*l1*m3*q11dot^2*sin(q11)*sin(q12)*sin(q13)))/6; (l1*(3*g*m1*cos(q21) - 3*lambda2*sin(q21) - 6*lambda3*cos(q21) + 6*g*m2*cos(q21) + 2*g*m3*cos(q21) + 3*3^(1/2)*lambda1*sin(q21) - 3*l2*m2*q22dot^2*cos(q22)*sin(q21) - 2*l2*m3*q22dot^2*cos(q22)*sin(q21) + 3*l2*m2*q22dot^2*cos(q21)*cos(q23)*sin(q22) + 3*l2*m2*q23dot^2*cos(q21)*cos(q23)*sin(q22) + 2*l2*m3*q22dot^2*cos(q21)*cos(q23)*sin(q22) + 2*l2*m3*q23dot^2*cos(q21)*cos(q23)*sin(q22) + 6*l2*m2*q22dot*q23dot*cos(q21)*cos(q22)*sin(q23) + 4*l2*m3*q22dot*q23dot*cos(q21)*cos(q22)*sin(q23)))/6; (l2*(3*3^(1/2)*lambda1*sin(q22) - 3*lambda2*sin(q22) - 6*lambda3*cos(q22)*cos(q23) + 3*lambda1*cos(q22)*sin(q23) + l2*m2*q23dot^2*sin(2*q22) + l2*m3*q23dot^2*sin(2*q22) + 3*g*m2*cos(q22)*cos(q23) + 2*g*m3*cos(q22)*cos(q23) + 3*3^(1/2)*lambda2*cos(q22)*sin(q23) - 3*l1*m2*q21dot^2*cos(q21)*sin(q22) - 2*l1*m3*q21dot^2*cos(q21)*sin(q22) + 3*l1*m2*q21dot^2*cos(q22)*cos(q23)*sin(q21) + 2*l1*m3*q21dot^2*cos(q22)*cos(q23)*sin(q21)))/6; -(l2*(3*g*m2*sin(q22)*sin(q23) - 6*lambda3*sin(q22)*sin(q23) - 3*3^(1/2)*lambda2*cos(q23)*sin(q22) - 3*lambda1*cos(q23)*sin(q22) + 2*g*m3*sin(q22)*sin(q23) + 2*l2*m2*q22dot*q23dot*sin(2*q22) + 2*l2*m3*q22dot*q23dot*sin(2*q22) + 3*l1*m2*q21dot^2*sin(q21)*sin(q22)*sin(q23) + 2*l1*m3*q21dot^2*sin(q21)*sin(q22)*sin(q23)))/6; (l1*(3*g*m1*cos(q31) - 3*lambda5*sin(q31) - 6*lambda6*cos(q31) + 6*g*m2*cos(q31) + 2*g*m3*cos(q31) - 3*3^(1/2)*lambda4*sin(q31) - 3*l2*m2*q32dot^2*cos(q32)*sin(q31) - 2*l2*m3*q32dot^2*cos(q32)*sin(q31) + 3*l2*m2*q32dot^2*cos(q31)*cos(q33)*sin(q32) + 3*l2*m2*q33dot^2*cos(q31)*cos(q33)*sin(q32) + 2*l2*m3*q32dot^2*cos(q31)*cos(q33)*sin(q32) + 2*l2*m3*q33dot^2*cos(q31)*cos(q33)*sin(q32) + 6*l2*m2*q32dot*q33dot*cos(q31)*cos(q32)*sin(q33) + 4*l2*m3*q32dot*q33dot*cos(q31)*cos(q32)*sin(q33)))/6; (l2*(3*lambda4*cos(q32)*sin(q33) - 3*3^(1/2)*lambda4*sin(q32) - 6*lambda6*cos(q32)*cos(q33) - 3*lambda5*sin(q32) + l2*m2*q33dot^2*sin(2*q32) + l2*m3*q33dot^2*sin(2*q32) + 3*g*m2*cos(q32)*cos(q33) + 2*g*m3*cos(q32)*cos(q33) - 3*3^(1/2)*lambda5*cos(q32)*sin(q33) - 3*l1*m2*q31dot^2*cos(q31)*sin(q32) - 2*l1*m3*q31dot^2*cos(q31)*sin(q32) + 3*l1*m2*q31dot^2*cos(q32)*cos(q33)*sin(q31) + 2*l1*m3*q31dot^2*cos(q32)*cos(q33)*sin(q31)))/6; -(l2*(3*3^(1/2)*lambda5*cos(q33)*sin(q32) - 6*lambda6*sin(q32)*sin(q33) - 3*lambda4*cos(q33)*sin(q32) + 3*g*m2*sin(q32)*sin(q33) + 2*g*m3*sin(q32)*sin(q33) + 2*l2*m2*q32dot*q33dot*sin(2*q32) + 2*l2*m3*q32dot*q33dot*sin(2*q32) + 3*l1*m2*q31dot^2*sin(q31)*sin(q32)*sin(q33) + 2*l1*m3*q31dot^2*sin(q31)*sin(q32)*sin(q33)))/6 ];

    %% Compute 'H' and 'Hdot'
    Hdot = [0, l2*q13dot*cos(q12)*cos(q13) - l2*q12dot*sin(q12)*sin(q13), l2*q12dot*cos(q12)*cos(q13) - l2*q13dot*sin(q12)*sin(q13), (3^(1/2)*l1*q21dot*cos(q21))/2, (l2*q23dot*cos(q22)*cos(q23))/2 - q22dot*((l2*sin(q22)*sin(q23))/2 - (3^(1/2)*l2*cos(q22))/2), (l2*q22dot*cos(q22)*cos(q23))/2 - (l2*q23dot*sin(q22)*sin(q23))/2, 0, 0, 0; -l1*q11dot*cos(q11), -l2*q12dot*cos(q12), 0, -(l1*q21dot*cos(q21))/2, (3^(1/2)*l2*q23dot*cos(q22)*cos(q23))/2 - q22dot*((l2*cos(q22))/2 + (3^(1/2)*l2*sin(q22)*sin(q23))/2), (3^(1/2)*l2*q22dot*cos(q22)*cos(q23))/2 - (3^(1/2)*l2*q23dot*sin(q22)*sin(q23))/2, 0, 0, 0; -l1*q11dot*sin(q11), - l2*q12dot*cos(q13)*sin(q12) - l2*q13dot*cos(q12)*sin(q13), - l2*q12dot*cos(q12)*sin(q13) - l2*q13dot*cos(q13)*sin(q12), l1*q21dot*sin(q21), l2*q22dot*cos(q23)*sin(q22) + l2*q23dot*cos(q22)*sin(q23), l2*q22dot*cos(q22)*sin(q23) + l2*q23dot*cos(q23)*sin(q22), 0, 0, 0; 0, l2*q13dot*cos(q12)*cos(q13) - l2*q12dot*sin(q12)*sin(q13), l2*q12dot*cos(q12)*cos(q13) - l2*q13dot*sin(q12)*sin(q13), 0, 0, 0, -(3^(1/2)*l1*q31dot*cos(q31))/2, (l2*q33dot*cos(q32)*cos(q33))/2 - q32dot*((l2*sin(q32)*sin(q33))/2 + (3^(1/2)*l2*cos(q32))/2), (l2*q32dot*cos(q32)*cos(q33))/2 - (l2*q33dot*sin(q32)*sin(q33))/2; -l1*q11dot*cos(q11), -l2*q12dot*cos(q12), 0, 0, 0, 0, -(l1*q31dot*cos(q31))/2, - q32dot*((l2*cos(q32))/2 - (3^(1/2)*l2*sin(q32)*sin(q33))/2) - (3^(1/2)*l2*q33dot*cos(q32)*cos(q33))/2, (3^(1/2)*l2*q33dot*sin(q32)*sin(q33))/2 - (3^(1/2)*l2*q32dot*cos(q32)*cos(q33))/2; -l1*q11dot*sin(q11), - l2*q12dot*cos(q13)*sin(q12) - l2*q13dot*cos(q12)*sin(q13), - l2*q12dot*cos(q12)*sin(q13) - l2*q13dot*cos(q13)*sin(q12), 0, 0, 0, l1*q31dot*sin(q31), l2*q32dot*cos(q33)*sin(q32) + l2*q33dot*cos(q32)*sin(q33), l2*q32dot*cos(q32)*sin(q33) + l2*q33dot*cos(q33)*sin(q32)];
    H = [0, l2*cos(q12)*sin(q13), l2*cos(q13)*sin(q12), (3^(1/2)*l1*sin(q21))/2, (3^(1/2)*l2*sin(q22))/2 + (l2*cos(q22)*sin(q23))/2, (l2*cos(q23)*sin(q22))/2, 0, 0, 0; -l1*sin(q11), -l2*sin(q12), 0, -(l1*sin(q21))/2, (3^(1/2)*l2*cos(q22)*sin(q23))/2 - (l2*sin(q22))/2, (3^(1/2)*l2*cos(q23)*sin(q22))/2, 0, 0, 0; l1*cos(q11), l2*cos(q12)*cos(q13), -l2*sin(q12)*sin(q13), -l1*cos(q21), -l2*cos(q22)*cos(q23), l2*sin(q22)*sin(q23), 0, 0, 0; 0, l2*cos(q12)*sin(q13), l2*cos(q13)*sin(q12), 0, 0, 0, -(3^(1/2)*l1*sin(q31))/2, (l2*cos(q32)*sin(q33))/2 - (3^(1/2)*l2*sin(q32))/2, (l2*cos(q33)*sin(q32))/2; -l1*sin(q11), -l2*sin(q12), 0, 0, 0, 0, -(l1*sin(q31))/2, - (l2*sin(q32))/2 - (3^(1/2)*l2*cos(q32)*sin(q33))/2, -(3^(1/2)*l2*cos(q33)*sin(q32))/2; l1*cos(q11), l2*cos(q12)*cos(q13), -l2*sin(q12)*sin(q13), 0, 0, 0, -l1*cos(q31), -l2*cos(q32)*cos(q33), l2*sin(q32)*sin(q33)];

    %% Compute the Raleigh dissipation function
    D = [d1; d2; d3; d1; d2; d3; d1; d2; d3];
    
    %% Solve for qddot numerically
    invM_times_b = invM * (b - D .* x(10:18));

    %% Perform Baumegarte index reduction
    z0 = [l2*sin(q12)*sin(q13) - (3^(1/2)*(r_base - r_platform + l1*cos(q21) + l2*cos(q22)))/2 + (l2*sin(q22)*sin(q23))/2; (3*r_base)/2 - (3*r_platform)/2 + l1*cos(q11) + l2*cos(q12) + (l1*cos(q21))/2 + (l2*cos(q22))/2 + (3^(1/2)*l2*sin(q22)*sin(q23))/2; l1*sin(q11) - l1*sin(q21) + l2*cos(q13)*sin(q12) - l2*cos(q23)*sin(q22); (3^(1/2)*(r_base - r_platform + l1*cos(q31) + l2*cos(q32)))/2 + l2*sin(q12)*sin(q13) + (l2*sin(q32)*sin(q33))/2; (3*r_base)/2 - (3*r_platform)/2 + l1*cos(q11) + l2*cos(q12) + (l1*cos(q31))/2 + (l2*cos(q32))/2 - (3^(1/2)*l2*sin(q32)*sin(q33))/2; l1*sin(q11) - l1*sin(q31) + l2*cos(q13)*sin(q12) - l2*cos(q33)*sin(q32)];
    z1 = [q22dot*((3^(1/2)*l2*sin(q22))/2 + (l2*cos(q22)*sin(q23))/2) + (3^(1/2)*l1*q21dot*sin(q21))/2 + l2*q12dot*cos(q12)*sin(q13) + l2*q13dot*cos(q13)*sin(q12) + (l2*q23dot*cos(q23)*sin(q22))/2; (3^(1/2)*l2*q23dot*cos(q23)*sin(q22))/2 - l1*q11dot*sin(q11) - l2*q12dot*sin(q12) - (l1*q21dot*sin(q21))/2 - q22dot*((l2*sin(q22))/2 - (3^(1/2)*l2*cos(q22)*sin(q23))/2); l1*q11dot*cos(q11) - l1*q21dot*cos(q21) - l2*q13dot*sin(q12)*sin(q13) + l2*q23dot*sin(q22)*sin(q23) + l2*q12dot*cos(q12)*cos(q13) - l2*q22dot*cos(q22)*cos(q23); l2*q12dot*cos(q12)*sin(q13) - (3^(1/2)*l1*q31dot*sin(q31))/2 - q32dot*((3^(1/2)*l2*sin(q32))/2 - (l2*cos(q32)*sin(q33))/2) + l2*q13dot*cos(q13)*sin(q12) + (l2*q33dot*cos(q33)*sin(q32))/2; - q32dot*((l2*sin(q32))/2 + (3^(1/2)*l2*cos(q32)*sin(q33))/2) - l1*q11dot*sin(q11) - l2*q12dot*sin(q12) - (l1*q31dot*sin(q31))/2 - (3^(1/2)*l2*q33dot*cos(q33)*sin(q32))/2; l1*q11dot*cos(q11) - l1*q31dot*cos(q31) - l2*q13dot*sin(q12)*sin(q13) + l2*q33dot*sin(q32)*sin(q33) + l2*q12dot*cos(q12)*cos(q13) - l2*q32dot*cos(q32)*cos(q33)];
    z2 = Hdot * x(10:18) + H * invM_times_b;

    z = z0 + z1 + z2; % Hurwitz polynomial

    %% Return the residuals for the solver
    out = [
        x(10:18); % first derivative residual
        invM_times_b; % second derivative residual 
        z % lagrange multiplier residuals
    ];
end