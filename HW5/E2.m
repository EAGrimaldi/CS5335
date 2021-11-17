% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the known initial vehicle state
% Output: x_est -> 1xT cell array containing the map state mean
%                  for T time steps (i.e., x_est{t} is a (2M)x1 vector,
%                  where M is the number of landmarks observed by time t)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a (2M)x(2M) matrix,
%                  where M is the number of landmarks observed by time t)
%         indices -> Mx1 vector containing the landmark index corresponding
%                    to the entries in the state vector, where M is the
%                    number of landmarks observed by the final time step T)
%                    For example, if indices is [15; 4], then the first two
%                    rows of x_est and P_est correspond to landmark 15,
%                    and the next two rows correspond to landmark 4, etc.

function [x_est, P_est, indices] = E2(odo, zind, z, W, x0)
    v = x0;
    x = [];
    P = [];
    indices = [];
    T = length(zind);
    x_est = cell(1,T);
    P_est = cell(1,T);
    for t = 1:T
        v = [v(1)+odo(1,t)*cos(v(3)); v(2)+odo(1,t)*sin(v(3)); v(3)+odo(2,t)];
        if zind(t)
            if ~ismember(zind(t),indices) % new landmark
                indices = [indices; zind(t)];
                x = [x;
                     v(1)+z{t}(1)*cos(v(3)+z{t}(2));
                     v(2)+z{t}(1)*sin(v(3)+z{t}(2))]; % PC 6.18
                Gz = [cos(v(3)+z{t}(2)) -z{t}(1)*sin(v(3)+z{t}(2));
                      sin(v(3)+z{t}(2)) z{t}(1)*cos(v(3)+z{t}(2))]; % PC 6.22
                Yz = blkdiag(eye(size(P,1)),Gz); % PC 6.20 (from PC 6.21 Gx is all zeroes)
                P = Yz*blkdiag(P,W)*Yz.'; % PC 6.19
            else % old landmark
                i = find(indices==zind(t));
                diff = [x(i*2-1); x(i*2)] - v(1:2);
                r = norm(diff);
                b = PCangdiff(atan2(diff(2), diff(1)), v(3));
                z_pred = [r; b]; % PC 6.8 more or less
                
                inno = z{t}-z_pred; % PC 6.9

                Hpi = [diff(1)/r diff(2)/r; -diff(2)/(r^2) diff(1)/(r^2)]; % PC 6.23
                m = size(indices,1);
                Hx = [zeros(2,2*(i-1)) Hpi zeros(2,2*(m-i))]; % PC 6.24
                Hw = eye(2); % PC 6.15
                S = Hx*P*Hx.' + Hw*W*Hw.'; % PC 6.13
                K = P*Hx.'*inv(S); % PC 6.12
  
                x = x + K*inno; % PC 6.10
                P = P - K*Hx*P; % PC 6.11
            end
        end
        x_est{t} = x;
        P_est{t} = P;
    end  
end