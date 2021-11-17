% Input: robot -> A 2-DOF robot encapsulated in a MATLAB cell with fields:
%                 robot.link1, robot.link2, robot.pivot1, robot.pivot2
%                 See description in hw2_cspace and demo code in C1.
%        obstacles -> 1xN vector of polyshape objects describing N 2-D
%                     polygonal obstacles
%        q_grid -> 1xN vector of angles between 0 and 2*pi, discretizing
%                  each dimension of configuration space
% Output: cspace -> NxN matrix: cspace(i,j)
%                   == 1 if [q_grid(i); q_grid(j)] is in collision,
%                   == 0 otherwise

function cspace = C2(robot, obstacles, q_grid)
    [~, res] = size(q_grid);
    cspace = zeros(res, res);
	for i = 1:res
        for j = 1:res
            [poly1, poly2, ~, ~] = q2poly(robot, [q_grid(1,i); q_grid(1,j)]);
            for obs = obstacles
                col1 = overlaps(obs, poly1);
                col2 = overlaps(obs, poly2);
                if col1 || col2
                   cspace(i,j) = 1;
                   break
                end
            end
        end
	end
end