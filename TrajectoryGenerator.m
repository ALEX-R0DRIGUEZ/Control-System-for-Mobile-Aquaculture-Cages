function [psi, psi_d, r_d] = TrajectoryGenerator(n, wpt, u, v)
% -------------------------------------------------------------------------
% TrajectoryGenerator computes the desired heading angle (psi_d) and the 
% desired yaw rate (r_d) when following a straight path with waypoints
% (wpt.pos.x, wpt.pos.y) every 50 meters.
% 
% Inputs:
%     n = [x, y, psi]': crafts North-East-Down (NED) positions
%                    x: Eastward position (m)
%                    y: Northward position (m)
%                  psi: Heading direction / rotation about the z-axis (rad)
%     wpt.pos.x = [x1, x2,..,xn]': array of waypoints expressed in NED (m)
%     wpt.pos.y = [y1, y2,..,yn]': array of waypoints expressed in NED (m)
%     u: surge speed (m/s)
%     v: sway speed (m/s)
% 
% Outputs:
%     psi: Heading direction (rad) with origin heading Eastward
%     psi_d: Desired heading direction (rad)
%     r_d: Desired yaw rate (rad/s)
% -------------------------------------------------------------------------

persistent k;       % active waypoint index
persistent xk yk;   % active waypoint (xk, yk) corresponding to integer k

R_switch = 5;       % threshold for switching to next waypoint (m)

%% Initialization of (xk, yk)
if isempty(k)
    % check if R_switch is smaller than the minimum distance between the waypoints
    if R_switch > min( sqrt( diff(wpt.pos.x).^2 + diff(wpt.pos.y).^2 ) )
        error("The distances between the waypoints must be larger than R_switch");
    end 
    
    % set first active waypoint
    k = 2;
    xk = wpt.pos.x(k);
    yk = wpt.pos.y(k);
end

%% Read next and previous waypoints (xk_next, yk_next) and from (xk_prev, yk_prev) wpt.pos 
K = length(wpt.pos.x);  % Number of waypoints

if k < K                % if there are more waypoints, read next and previous waypoints  
    xk_next = wpt.pos.x(k+1);  
    yk_next = wpt.pos.y(k+1);
    xk_prev = wpt.pos.x(k-1);
    yk_prev = wpt.pos.y(k-1);
else                    % else, on last waypoint so continue along last bearing
    bearing = atan2((wpt.pos.y(K)-wpt.pos.y(K-1)), (wpt.pos.x(K)-wpt.pos.x(K-1)));
    % Make next waypoint along same bearing but far away so craft continues
    % in that direction
    R = 1e10;
    xk_next = wpt.pos.x(K) + R * cos(bearing);
    yk_next = wpt.pos.y(K) + R * sin(bearing); 
    xk_prev = wpt.pos.x(K-1);
    yk_prev = wpt.pos.y(K-1);
end

% Print active waypoint 
fprintf('Active waypoint:\n')
fprintf('  (x%1.0f, y%1.0f) = (%.2f, %.2f) \n',k,k,xk,yk);

%% Compute desired heading and desired yaw rate

% Compute the desired course angle w.r.t. East/positive x direction
theta_d = atan2( (yk-yk_prev), (xk-xk_prev) );

% Extract x, y, and psi from n
x = n(1);
y = n(2);
psi = wrapToPi(n(3));

% Along-path error (+ value = ahead of current waypoint in path direction)
AP_err =  (x-xk) * cos(theta_d) + (y-yk) * sin(theta_d);

% Cross-path error (+ value = right of path w.r.t path direction)
CP_err = (x-xk) * sin(theta_d) - (y-yk) * cos(theta_d);

% Waypoint switching criterion, k = k + 1
% If craft's position is within R_switch away from waypoint in path
% direction and within 2*R_switch away from waypoint perpendicular to path
% then switch to next waypoint
if AP_err > -R_switch && abs(CP_err) < (2*R_switch)
    if k < K                        % if not on last waypoint
        k = k + 1;
        xk = xk_next;               % update active waypoint
        yk = yk_next;
    else                            % else, on last waypoint
        xk = xk_next;               % update active waypoint
        yk = yk_next;
        xk_prev = wpt.pos.x(K);     % Set last waypoint in array as previous waypoint
        yk_prev = wpt.pos.y(K);
    end
end

% Desired heading (psi) to move towards next waypoint
psi_d = atan2((yk-y),(xk-x));

% Distance to next waypoint
delta = sqrt((yk-y)^2 + (xk-x)^2);

Kp = 1/delta;

% Speed magnitude of craft
V_body = sqrt(u^2 + v^2);

% Desired d/dt psi = desired yaw (yaw = r, in BODY cooridinates)
% Allows for slower turns near path to reduce jitteriness
D_CP_err = -V_body * CP_err / sqrt( delta^2 + CP_err^2 );    % d/dt CP_err
r_d = -Kp * D_CP_err / ( 1 + (Kp * CP_err)^2 );              % d/dt psi_d

end