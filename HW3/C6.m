% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_path -> Mx2 matrix containing a collision-free path from
%                  q_start to q_goal. Each row in q_path is a robot
%                  configuration. The first row should be q_start,
%                  the final row should be q_goal.
% Output: num_collisions -> Number of swept-volume collisions encountered
%                           between consecutive configurations in q_path

function num_collisions = C6(robot, obstacles, q_path)
    [length,~] = size(q_path);
    num_collisions=0;
    for i=1:length-1
        [poly1i,poly2i,~,~] = q2poly(robot,q_path(i,:).');
        [poly1j,poly2j,~,~] = q2poly(robot,q_path(i+1,:).');
        poly1 = union(poly1i,poly1j);
        poly1 = convhull(poly1);
        poly2 = union(poly2i,poly2j);
        poly2 = convhull(poly2);
        for obs = obstacles
            col1 = overlaps(obs, poly1);
            if col1
                num_collisions=num_collisions+1;
            end
            col2 = overlaps(obs, poly2);
            if col2
                num_collisions=num_collisions+1;
            end
            if col1||col2
                plot(poly1, 'FaceColor', 'r')
                plot(poly2, 'FaceColor', 'b')
            end
        end
    end
end