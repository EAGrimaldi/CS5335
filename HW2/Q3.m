% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector describing goal position
%        epsilon -> scalar denoting how close end effector must be before
%                   loop terminates
%        velocity -> scalar denoting desired velocity of end effector
% output: traj -> nx9 matrix that denotes arm trajectory. Each row denotes
%                 a joint configuration. The first row is the first
%                 configuration. The last is the last configuration in the
%                 trajectory.

function traj = Q3(f, qInit, posGoal, epsilon, velocity)
    curr_q = qInit;
    delta = epsilon+1;
    %pos = zeros(3,1000);
    traj = zeros(1000,9);
    traj(1,:) = qInit;
    i = 1;
    while(delta > epsilon)
        curr_pos = f.fkine(curr_q).t;
        %pos(:,i) = curr_pos;
        dx = posGoal - curr_pos;
        dx_vel = velocity*dx/norm(dx);
        J = f.jacob0(curr_q);
        Jpinv = pinv(J(1:3,:));
        dq = Jpinv * dx_vel;
        curr_q = curr_q + dq.';
        traj(i+1,:) = curr_q;
        delta = norm(dx);
        i = i+1;
    end
    traj = traj(1:i,:);
    %pos = pos(:,1:i);
    %save('pos.mat','pos');
end