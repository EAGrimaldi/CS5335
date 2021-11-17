% Input: q_min -> 1x4 vector of minimum angle for each joint
%        q_max -> 1x4 vector of maximum angle for each joint
%        num_samples -> Integer denoting number of samples to sample
% Output: qs -> num_samples x 4 matrix of joint angles,
%               all within joint limits

function qs = M1(q_min, q_max, num_samples)
    qs = repmat(q_min, num_samples, 1) + repmat(q_max-q_min, num_samples, 1) .* rand(num_samples, 4);
end