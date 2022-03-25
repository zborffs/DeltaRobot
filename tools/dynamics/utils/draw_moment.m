function pplot = draw_moment(x11, y11, z11, x12, y12, z12, x21, y21, z21, x22, y22, z22, x31, y31, z31, x32, y32, z32, p)
% draw_moment draws a snapshot of the delta robot using MATLAB plotting
% features 

%% Create temporary variables
v = rot_z(-2 * pi / 3) * [x21; y21; z21]; x21 = v(1); y21 = v(2); z21 = v(3);
v = rot_z(-2 * pi / 3) * [x22; y22; z22]; x22 = v(1); y22 = v(2); z22 = v(3);
v = rot_z(2 * pi / 3) * [x31; y31; z31]; x31 = v(1); y31 = v(2); z31 = v(3);
v = rot_z(2 * pi / 3) * [x32; y32; z32]; x32 = v(1); y32 = v(2); z32 = v(3);

%% Make individual plots
plot3([0, -x11], [-p.r_base, -y11], [0.0, -z11], '-o', 'Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF'); hold on;
plot3([-x11, -x12], [-y11, -y12], [-z11, -z12], '-o', 'Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF'); hold on;
plot3([-p.r_base * -sin(-2 * pi / 3),-x21], [-p.r_base * cos(-2 * pi / 3),-y21], [0, -z21], '-o', 'Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF'); hold on;
plot3([-x21,-x22], [-y21,-y22], [-z21, -z22], '-o', 'Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF'); hold on;
plot3([-p.r_base * -sin(2 * pi / 3), -x31], [-p.r_base * cos(2 * pi / 3),-y31], [0, -z31], '-o', 'Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF'); hold on;

%% Return aggregated plot
pplot = plot3([-x31,-x32], [-y31,-y32], [-z31, -z32], '-o', 'Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF'); xlim([-0.5, 0.5]); ylim([-0.5, 0.5]); zlim([-1.1, 0]); hold off;

end

