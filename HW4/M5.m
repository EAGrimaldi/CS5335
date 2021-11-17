% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        path -> Nx4 matrix containing a collision-free path between
%                q_start and q_goal
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: smoothed_path -> Nx4 matrix containing a smoothed version of the
%                          input path, where some unnecessary intermediate
%                          waypoints may have been removed

function smoothed_path = M5(robot, path, link_radius, sphere_centers, sphere_radii)
    length = size(path, 1);
    smoothed_path = zeros(length, 4);
    smoothed_path(1,:) = path(1,:);
    curr = 1;
    i = 2;
    while curr<length
        test = length;
        while test>curr
            if test==curr+1
                smoothed_path(i,:) = path(test,:); 
                curr = test;
                i = i+1;
                break;
            elseif ~check_edge(robot, path(curr,:), path(test,:), link_radius, sphere_centers, sphere_radii)
                smoothed_path(i,:) = path(test,:); 
                curr = test;
                i = i+1;
                break;
            end
            test = test-1;
        end
    end
    smoothed_path = smoothed_path(1:i-1,:);
end

