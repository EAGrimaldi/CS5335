% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to plot the robot at
% Output: plots the robot links and pivots at the provided input configuration.

function C1(robot, q)  
    % Transform frame origins and compute link polygon corners
    [link1_at0, link2_at0, origin1_at0, origin2_at0] = q2poly(robot, q);
    % Plot the links
    plot(link1_at0, 'FaceColor', 'r');
    plot(link2_at0, 'FaceColor', 'b');
    % Plot the pivot points
    plot(origin1_at0(1), origin1_at0(2), 'k.', 'MarkerSize', 10);
    plot(origin2_at0(1), origin2_at0(2), 'k.', 'MarkerSize', 10);
end