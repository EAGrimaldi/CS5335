% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        samples -> num_samples x 4 matrix, vertices in the roadmap
%        adjacency -> num_samples x num_samples matrix, the weighted
%                     adjacency matrix denoting edges in the roadmap
%        q_start -> 1x4 vector denoting the start configuration
%        q_goal -> 1x4 vector denoting the goal configuration
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: path -> Nx4 matrix containing a collision-free path between
%                 q_start and q_goal, if a path is found. The first row
%                 should be q_start, the final row should be q_goal.
%         path_found -> Boolean denoting whether a path was found

function [path, path_found] = M3(robot, samples, adjacency, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    [knn_index, ~] = knnsearch(samples, [q_start; q_goal], 'K', 10);
    % must find "on ramp" and "off ramp" that we can connect with roadmap
    % since q_start and q_goal are probably not in samples.
    % knnsearch finds 10 candidate on ramps and off ramps
    % knnsearch organizes the returned NNs by distance to the query points
    % we then choose the ramps that have the shortest non-colliding edge
    % to the query points. Hopefully 10 NNs is sufficient lol.
    for k=1:10
        q_maybe = samples(knn_index(1,k),:);
        if ~check_edge(robot, q_start, q_maybe, link_radius, sphere_centers, sphere_radii)
            q_on_index = knn_index(1,k);
            break
        end
    end
    for k=1:10
        q_maybe = samples(knn_index(2,k),:);
        if ~check_edge(robot, q_goal, q_maybe, link_radius, sphere_centers, sphere_radii)
            q_off_index = knn_index(2,k);
            break
        end
    end
    path_index = shortestpath(digraph(adjacency), q_on_index, q_off_index);
    path_size = size(path_index,2);
    path = zeros(path_size,4);
    for n=1:path_size
        path(n,:) = samples(path_index(n),:);
    end
    path = cat(1, q_start, path);
    path = cat(1, path, q_goal);
    if isempty(path_index)
        path_found = false;
    else
        path_found = true;
    end
end