% Input: robot -> A 4-DOF robot encoded as a SerialLink object
%        q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of desired samples
%        link_radius -> Scalar denoting radius of each robot link's
%                       cylindrical body
%        sphere_centers -> Nx3 matrix containing the centers of N spherical
%                          obstacles
%        sphere_radii -> Nx1 vector containing the radii of N spherical
%                        obstacles
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits and not in collision

function qs = M1point5(robot, q_min, q_max, num_samples, link_radius, sphere_centers, sphere_radii)
    qs = zeros(num_samples, 4);
    for i=1:num_samples
        sample = q_min + (q_max-q_min).*rand(1, 4);
        while (check_collision(robot, sample, link_radius, sphere_centers, sphere_radii))
            sample = q_min + (q_max-q_min).*rand(1, 4);
        end
        qs(i,:) = sample;
    end
end