% input: f -> a 9-joint robot encoded as a SerialLink class
%        qInit -> 1x9 vector denoting current joint configuration
%        posGoal -> 3x1 vector denoting the target position to move to
% output: q -> 1x9 vector of joint angles that cause the end
%              effector position to reach <position>
%              (orientation is to be ignored)

function q = Q2(f, qInit, posGoal)
    curr_q = qInit;
    step = 0.01;
    delta = 1;
    while(delta > 0.01)
        curr_pos = f.fkine(curr_q).t;
        dx = posGoal - curr_pos;
        J = f.jacob0(curr_q);
        Jpinv = pinv(J(1:3,:));
        dq = step * Jpinv * dx;
        curr_q = curr_q + dq.';
        delta = norm(dx);
    end
	q = curr_q;
end