% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
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

function [path, path_found] = M4(robot, q_min, q_max, q_start, q_goal, link_radius, sphere_centers, sphere_radii)
    step_size = 0.5;
    sample_goal = 10;
    max_samples = 10000;
    samples = zeros(max_samples,4);
    adjacency = zeros(max_samples);
    samples(1,:) = q_start;
    n=2;
    k=1;
    path_found = false;
    goal_index = 0;
    while n<max_samples
        if k<sample_goal
            sample = q_min + (q_max-q_min).*rand(1, 4);
            while (check_collision(robot, sample, link_radius, sphere_centers, sphere_radii))
                sample = q_min + (q_max-q_min).*rand(1, 4);
            end
            path_found = false;
            k = k+1;
        else
            sample = q_goal;
            path_found = true;
            k = 1;
        end
        [nn_index, nn_dist] = knnsearch(samples(1:n-1,:), sample);
        % using knnsearch to find the NN already in tree to the new sample
        nearest = samples(nn_index,:);
        dist = nn_dist;
        if dist>step_size
            sample = nearest + step_size*norm(sample-nearest);
            dist = step_size;
            if path_found
                path_found = false;
            end
        end
        [in_collision, free_dist] = check_edge_dist(robot, nearest, sample, link_radius, sphere_centers, sphere_radii);
        if in_collision
            sample = nearest + free_dist*norm(sample-nearest);
            dist = free_dist;
            if path_found
                path_found = false;
            end
        end
        if dist>0
            samples(n,:) = sample;
            adjacency(nn_index, n) = dist;
            if path_found
                goal_index = n;
                break
            end
            n = n+1;
        end
    end
    if path_found
        path_index = shortestpath(digraph(adjacency), 1, goal_index);
        path_size = size(path_index,2);
        path = zeros(path_size,4);
        for n=1:path_size
            path(n,:) = samples(path_index(n),:);
        end
    else
        path = [];
    end
end