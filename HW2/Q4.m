% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        circle -> 3xn matrix of Cartesian positions that describe the
%                  circle
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory. The end effector should trace a circle in the
%                 workspace, as specified by the input argument circle.
%                 (orientation is to be ignored)

function traj = Q4(f, qInit, circle, velocity)
    curr_q = qInit;
    d = size(circle);
    n = d(1, 2);
    traj = zeros(1000,9);
    traj(1,:) = qInit;
    j = 1;
    i = 1;
    while(i < n)
        next_pos = circle(:,i+1);
        sub_traj = Q3(f, curr_q, next_pos, 0.05, velocity);
        curr_q = sub_traj(end,:);
        sub_d = size(sub_traj);
        sub_n = sub_d(1,1);
        traj(j+1:j+sub_n,:)=sub_traj;
        j = j+sub_n;
        i = i+1;
    end
    traj = traj(1:j,:);
end