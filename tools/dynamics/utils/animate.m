function h = animate(t, y, p)
%animate(t, y, p) animates the solution returned by the DAE solver as a GIF

%% transform the spherical coordinates returned by solver to cartesian
[x11, y11, z11, x12, y12, z12] = transform2cartesian(t, y(:,1), y(:,2), y(:,3), p);
[x21, y21, z21, x22, y22, z22] = transform2cartesian(t, y(:,4), y(:,5), y(:,6), p);
[x31, y31, z31, x32, y32, z32] = transform2cartesian(t, y(:,7), y(:,8), y(:,9), p);

%% Evenly space out the 't' vector so animation tracks time linearly
t_step_init = 0.025;
t_step = t_step_init;
i = 1;
v = [];
while i < size(t, 1) - 1
    t_step = t_step - (t(i+1) - t(i));
    if t_step < 0
        v = [v; i];
        t_step = t_step_init;
    end
    i = i + 1;
end

%% Create the same number of frames as there are 't' samples
frames = size(v,1);

%% Create the 'Movie' object
Movie(frames) = struct('cdata', [], 'colormap', []);

%% Take several snapshot images of the plot and combine them into gif
h = figure;
pplot = draw_moment(x11(1), y11(1), z11(1), x12(1), y12(1), z12(1), x21(1), y21(1), z21(1), x22(1), y22(1), z22(1), x31(1), y31(1), z31(1), x32(1), y32(1), z32(1), p);
axis tight manual; % apply axis restrictions
ax = gca;
ax.NextPlot = 'replaceChildren';

filename = 'delta_robot_animation.gif';

for j = 1:frames
    draw_moment(x11(v(j)), y11(v(j)), z11(v(j)), ...
        x12(v(j)), y12(v(j)), z12(v(j)), ...
        x21(v(j)), y21(v(j)), z21(v(j)), ...
        x22(v(j)), y22(v(j)), z22(v(j)), ...
        x31(v(j)), y31(v(j)), z31(v(j)), ...
        x32(v(j)), y32(v(j)), z32(v(j)), p);
    grid on;
    title("t="+t(v(j)))

    drawnow
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    if j == 1 
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime', 0.05); 
    else 
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.05); 
    end 
end
end