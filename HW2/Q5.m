% input: f1 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        f2 -> an 9-joint robot encoded as a SerialLink class for one
%              finger
%        qInit -> 1x11 vector denoting current joint configuration.
%                 First seven joints are the arm joints. Joints 8,9 are
%                 finger joints for f1. Joints 10,11 are finger joints
%                 for f2.
%        f1Target, f2Target -> 3x1 vectors denoting the target positions
%                              each of the two fingers.
% output: q -> 1x11 vector of joint angles that cause the fingers to
%              reach the desired positions simultaneously.
%              (orientation is to be ignored)

function q = Q5(f1, f2, qInit, f1Target, f2Target)
    curr_q = qInit;
	step = 0.01;
    delta = 1;
    while(delta > 0.01)
        curr_q1 = curr_q(:,1:9);
        curr_q2 = [curr_q(:,1:7) curr_q(:,10:11)];
        curr_pos1 = f1.fkine(curr_q1).t;
        curr_pos2 = f2.fkine(curr_q2).t;
        dx1 = f1Target - curr_pos1;
        dx2 = f2Target - curr_pos2;
        dx = [dx1 ; dx2];
        J1 = f1.jacob0(curr_q1);
        J2 = f2.jacob0(curr_q2);
        J = [ J1(1:3, 1:7) J1(1:3, 8:9) zeros(3,2);
              J2(1:3, 1:7) zeros(3,2) J2(1:3, 8:9)];
        Jpinv = pinv(J);
        dq = step * Jpinv * dx;
        curr_q = curr_q + dq.';
        delta = norm(dx);
    end
	q = curr_q;
end