function L = lagrangian(t, y, p)
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

L = zeros(size(t));
g = p.g;
m1 = p.m1;

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

q11 = y(:,1);
q12 = y(:,2);
q13 = y(:,3);
q21 = y(:,4);
q22 = y(:,5);
q23 = y(:,6);
q31 = y(:,7);
q32 = y(:,8);
q33 = y(:,9);
q11dot = y(:,10);
q12dot = y(:,11);
q13dot = y(:,12);
q21dot = y(:,13);
q22dot = y(:,14);
q23dot = y(:,15);
q31dot = y(:,16);
q32dot = y(:,17);
q33dot = y(:,18);

% extract the six lagrange multipliers from the 'x' argument
lambda1 = y(:,19);
lambda2 = y(:,20);
lambda3 = y(:,21);
lambda4 = y(:,22);
lambda5 = y(:,23);
lambda6 = y(:,24);

for i = 1:length(L)
    Li = g*((l1*sin(q11(i))*(3*m1 + 6*m2 + 2*m3))/6 + (l2*cos(q13(i))*sin(q12(i))*(3*m2 + 2*m3))/6) + g*((l1*sin(q21(i))*(3*m1 + 6*m2 + 2*m3))/6 + (l2*cos(q23(i))*sin(q22(i))*(3*m2 + 2*m3))/6) + g*((l1*sin(q31(i))*(3*m1 + 6*m2 + 2*m3))/6 + (l2*cos(q33(i))*sin(q32(i))*(3*m2 + 2*m3))/6) + lambda2(i)*((3*r_base)/2 - (3*r_platform)/2 + l1*cos(q11(i)) + l2*cos(q12(i)) + (l1*cos(q21(i)))/2 + (l2*cos(q22(i)))/2 + (3^(1/2)*l2*sin(q22(i))*sin(q23(i)))/2) + lambda5(i)*((3*r_base)/2 - (3*r_platform)/2 + l1*cos(q11(i)) + l2*cos(q12(i)) + (l1*cos(q31(i)))/2 + (l2*cos(q32(i)))/2 - (3^(1/2)*l2*sin(q32(i))*sin(q33(i)))/2) + (m3*(l1*q11dot(i)*cos(q11(i)) - l2*q13dot(i)*sin(q12(i))*sin(q13(i)) + l2*q12dot(i)*cos(q12(i))*cos(q13(i)))^2)/6 + (m2*(2*l1*q11dot(i)*cos(q11(i)) - l2*q13dot(i)*sin(q12(i))*sin(q13(i)) + l2*q12dot(i)*cos(q12(i))*cos(q13(i)))^2)/8 + (m3*(l1*q21dot(i)*cos(q21(i)) - l2*q23dot(i)*sin(q22(i))*sin(q23(i)) + l2*q22dot(i)*cos(q22(i))*cos(q23(i)))^2)/6 + (m2*(2*l1*q21dot(i)*cos(q21(i)) - l2*q23dot(i)*sin(q22(i))*sin(q23(i)) + l2*q22dot(i)*cos(q22(i))*cos(q23(i)))^2)/8 + (m3*(l1*q31dot(i)*cos(q31(i)) - l2*q33dot(i)*sin(q32(i))*sin(q33(i)) + l2*q32dot(i)*cos(q32(i))*cos(q33(i)))^2)/6 + (m2*(2*l1*q31dot(i)*cos(q31(i)) - l2*q33dot(i)*sin(q32(i))*sin(q33(i)) + l2*q32dot(i)*cos(q32(i))*cos(q33(i)))^2)/8 + lambda3(i)*(l1*sin(q11(i)) - l1*sin(q21(i)) + l2*cos(q13(i))*sin(q12(i)) - l2*cos(q23(i))*sin(q22(i))) + lambda6(i)*(l1*sin(q11(i)) - l1*sin(q31(i)) + l2*cos(q13(i))*sin(q12(i)) - l2*cos(q33(i))*sin(q32(i))) - lambda1(i)*((3^(1/2)*r_base)/2 - (3^(1/2)*r_platform)/2 - l2*sin(q12(i))*sin(q13(i)) - (l2*sin(q22(i))*sin(q23(i)))/2 + (3^(1/2)*l1*cos(q21(i)))/2 + (3^(1/2)*l2*cos(q22(i)))/2) + lambda4(i)*((3^(1/2)*r_base)/2 - (3^(1/2)*r_platform)/2 + l2*sin(q12(i))*sin(q13(i)) + (l2*sin(q32(i))*sin(q33(i)))/2 + (3^(1/2)*l1*cos(q31(i)))/2 + (3^(1/2)*l2*cos(q32(i)))/2) + (m3*(l1*q11dot(i)*sin(q11(i)) + l2*q12dot(i)*sin(q12(i)))^2)/6 + (m2*(2*l1*q11dot(i)*sin(q11(i)) + l2*q12dot(i)*sin(q12(i)))^2)/8 + (m3*(l1*q21dot(i)*sin(q21(i)) + l2*q22dot(i)*sin(q22(i)))^2)/6 + (m2*(2*l1*q21dot(i)*sin(q21(i)) + l2*q22dot(i)*sin(q22(i)))^2)/8 + (m3*(l1*q31dot(i)*sin(q31(i)) + l2*q32dot(i)*sin(q32(i)))^2)/6 + (m2*(2*l1*q31dot(i)*sin(q31(i)) + l2*q32dot(i)*sin(q32(i)))^2)/8 + (l2^2*m2*(q12dot(i)^2 + q13dot(i)^2*sin(q12(i))^2))/6 + (l2^2*m2*(q22dot(i)^2 + q23dot(i)^2*sin(q22(i))^2))/6 + (l2^2*m2*(q32dot(i)^2 + q33dot(i)^2*sin(q32(i))^2))/6 + (l1^2*m1*q11dot(i)^2)/6 + (l1^2*m1*q21dot(i)^2)/6 + (l1^2*m1*q31dot(i)^2)/6 + (l2^2*m2*(q12dot(i)*cos(q12(i))*sin(q13(i)) + q13dot(i)*cos(q13(i))*sin(q12(i)))^2)/8 + (l2^2*m3*(q12dot(i)*cos(q12(i))*sin(q13(i)) + q13dot(i)*cos(q13(i))*sin(q12(i)))^2)/6 + (l2^2*m2*(q22dot(i)*cos(q22(i))*sin(q23(i)) + q23dot(i)*cos(q23(i))*sin(q22(i)))^2)/8 + (l2^2*m3*(q22dot(i)*cos(q22(i))*sin(q23(i)) + q23dot(i)*cos(q23(i))*sin(q22(i)))^2)/6 + (l2^2*m2*(q32dot(i)*cos(q32(i))*sin(q33(i)) + q33dot(i)*cos(q33(i))*sin(q32(i)))^2)/8 + (l2^2*m3*(q32dot(i)*cos(q32(i))*sin(q33(i)) + q33dot(i)*cos(q33(i))*sin(q32(i)))^2)/6 + (l1^2*m1*q11dot(i)^2*cos(q11(i))^2)/8 + (l1^2*m1*q21dot(i)^2*cos(q21(i))^2)/8 + (l1^2*m1*q31dot(i)^2*cos(q31(i))^2)/8 + (l1^2*m1*q11dot(i)^2*sin(q11(i))^2)/8 + (l1^2*m1*q21dot(i)^2*sin(q21(i))^2)/8 + (l1^2*m1*q31dot(i)^2*sin(q31(i))^2)/8;
    L(i) = Li;
end


end

