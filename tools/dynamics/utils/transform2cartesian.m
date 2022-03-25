function [x1, y1, z1, x2, y2, z2] = transform2cartesian(t, phi1, phi2, phi3, p)
% transform2cartesian(t, phi1, phi2, phi3, p) transforms the solution to
% the DAE from spherical coordinates to cartesian coordinates

    %% Extract delta robot parameters
    l1 = p.l1;
    l2 = p.l2;
    lc1 = l1 / 2.0;
    lc2 = l2 / 2.0;

    %% end of link 1
    x1 = zeros(size(t));
    y1 = l1 * cos(phi1) + p.r_base;
    z1 = l1 * sin(phi1);

    %% end of link 2 (end-effector)
    x2 = l2 * sin(phi2) .* sin(phi3);
    y2 = y1 + l2 * cos(phi2);
    z2 = z1 + l2 * sin(phi2) .* cos(phi3);
end

