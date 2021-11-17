% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw3_cspace and demo code in C1.
%        q -> 2x1 vector denoting the configuration to convert to polygons
% Output: poly1 -> A polyshape object denoting the 2-D polygon describing
%                  link 1
%         poly2 -> A polyshape object denoting the 2-D polygon describing
%                  link 2
%         pivot1 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 1 (frame 1 origin), with respect to base frame
%         pivot2 -> 2x1 vector denoting the 2-D position of the pivot point
%                   of link 2 (frame 2 origin), with respect to base frame

function [poly1, poly2, pivot1, pivot2] = q2poly(robot, q)
    q1 = q(1,1);
    q2 = q(2,1);
    Rq1 = rot2(q1);
    Rq2 = rot2(q2);
    pivot1 = robot.pivot1;
    pivot2 = robot.pivot1 +  Rq1*robot.pivot2;
    %I elected to do this the "hard" way:
    T1to0 = [Rq1 robot.pivot1; zeros(1,2) 1];
    T2to1 = [Rq2 robot.pivot2; zeros(1,2) 1];
    T2to0 = T1to0*T2to1;
    poly1 = T1to0*[robot.link1; ones(1,4)];
    poly2 = T2to0*[robot.link2; ones(1,4)];
    poly1 = polyshape(poly1(1:2, :).');
    poly2 = polyshape(poly2(1:2, :).');
end