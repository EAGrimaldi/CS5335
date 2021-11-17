% Input: odo -> 2xT matrix containing odometry readings for T time steps
%        zind -> 1xT vector containing the observed landmark index for
%                T time steps; index is 0 if no landmark observed
%        z -> 1xT cell array containing the (range, bearing) observation
%             for T time steps; z{t} is empty if no observation at time t
%        V -> 2x2 matrix denoting the process noise in (forward, angular)
%        W -> 2x2 matrix denoting the sensing noise in (range, bearing)
%        x0 -> 3x1 vector denoting the initial vehicle state mean
%        P0 -> 3x3 matrix denoting the initial vehicle state covariance
%        map -> Robotics toolbox Map object containing the known map
%               (known landmarks) for localization
% Output: x_est -> 1xT cell array containing the vehicle state mean
%                  for T time steps (i.e., x_est{t} is a 3x1 vector)
%         P_est -> 1xT cell array containing the vehicle state covariance
%                  for T time steps (i.e., P_est{t} is a 3x3 matrix)

function [x_est, P_est] = E1(odo, zind, z, V, W, x0, P0, map)
    x = x0;
    P = P0;
    T = length(zind);
    x_est = cell(1,T);
    P_est = cell(1,T);  
    for t = 1:T       
        x_pred = [x(1)+odo(1,t)*cos(x(3)); x(2)+odo(1,t)*sin(x(3)); x(3)+odo(2,t)]; % PC 6.2
        
        Fx = [1 0 -odo(1,t)*sin(x(3)); 0 1 odo(1,t)*cos(x(3)); 0 0 1]; % PC 6.5
        Fv = [cos(x(3)) 0; sin(x(3)) 0; 0 1]; % PC 6.6
        P_pred = Fx*P*Fx.' + Fv*V*Fv.'; % PC 6.4
        if zind(t)
            landmark = map.map(:,zind(t));
            diff = landmark - x_pred(1:2);
            r = norm(diff);
            b = PCangdiff(atan2(diff(2), diff(1)), x_pred(3));
            z_pred = [r; b]; % PC 6.8 more or less

            inno = z{t}-z_pred; % PC 6.9

            Hx = [-diff(1)/r -diff(2)/r 0; diff(2)/(r^2) -diff(1)/(r^2) -1]; % PC 6.14
            Hw = eye(2); % PC 6.15

            S = Hx*P_pred*Hx.' + Hw*W*Hw.'; % PC 6.13
            K = P_pred*Hx.'*inv(S); % PC 6.12

            x = x_pred + K*inno; % PC 6.10
            P = P_pred - K*Hx*P_pred; % PC 6.11
        else
            x = x_pred;
            P = P_pred;
        end 
        x_est{t} = x;
        P_est{t} = P;       
    end
end