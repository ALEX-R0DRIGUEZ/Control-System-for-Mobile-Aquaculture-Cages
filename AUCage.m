function [V_dot, n_dot] = AUCage(tau, n, V, V_c, theta_c)
% -------------------------------------------------------------------------
% AUCage computes d/dt V and d/dt n after control forces are applied to the
% marine craft, where V is the velocity vector of the craft in the non-inertial
% frame and n is the position vector of the craft in the inertial frame.
% Hydrodynamic equations of motion are used from the "Handbook of Marine Craft 
% Hydrodynamics and Motion Control" by Thor I. Fossen and simplified for a 3DOF
% system.
% 
% Inputs:
%     tau = [T_surge, T_sway, delta_yaw]': control forces and moment applied
%                                         to the craft (N, N, Nm)
% 
%     n = [x, y, psi]': marine crafts North-East-Down (NED) positions
%                    x: Eastward position (m)
%                    y: Northward position (m)
%                  psi: Heading direction / rotation about the z-axis (rad) 
% 
%     V = [u, v, r]': marine crafts body-fixed (BODY) velocities
%                  u: Surge (forward) velocity (m/s)
%                  v: Sway (Lateral) velocity (m/s)
%                  r: Yaw (rotation about the z-axis) velocity (rad/s)
% 
%     V_c: Ocean current speed (m/s)
%     theta_c: Ocean current direction (rad)
% 
% Outputs:
%     V_dot = d/dt V
%           = [d/dt u, d/dt v, d/dt r]': marine crafts acceleration (BODY)
% 
%     n_dot = d/dt n
%           = [d/dt x, d/dt y, d/dt psi]': marine crafts velocity (NED)
%                                          (Note: d/dt psi = r)
% -------------------------------------------------------------------------

% Extract x, y, and psi from n
x = n(1);
y = n(2);
psi = wrapToPi(n(3));

% Extract u, v, and r from V
u = V(1);
v = V(2);
r = V(3);

% Conversion matrix for converting from BODY to NED reference frames
J = [cos(psi), -sin(psi), 0;
     sin(psi),  cos(psi), 0;
         0,          0,     1];

% Equation to convert BODY velocities into NED velocites (d/dt n)
n_dot = J * V;

u_c = V_c * cos(theta_c - psi);     % Ocean Current surge velocity
v_c = V_c * sin(theta_c - psi);     % Ocean Current sway velocity

u_r = u - u_c;                      % Relative velocitys
v_r = v - v_c;

V_r = [u_r; v_r; r];                % Relative velocity vector

% Marine Craft (Cylindrical Cage) data
m = 200000;                         % Mass (kg)
radius = 10;                        % Radius of the cage (m)
Iz = 0.5 * m * radius^2;            % Moment of Inertia about the z-axis (kg*m^2)
rho = 1025;                         % Water density (kg/m^3)
L = 2*radius;                       % Length of the cage (m)
height = L;                         % Height of the cage (m)
A = pi*radius*height;               % Area of front (m^2)

% Drag coefficients
Cx = 0.5;
Cy = 0.5;
Cn = 0.05;

% Drag force equations
Xc = -0.5*rho*A*Cx*abs(u_r);        % Sway drag
Yc = -0.5*rho*A*Cy*abs(v_r);        % Surge drag
Nc = -0.5*rho*A*Cn*L*abs(r);        % Yaw drag

% Added Mass Inertia components
Xu = ((4/3)*rho*pi)*(L/2)^3;
Yv = Xu;
Nr = 0;

% Rigid-Body Mass Inertia Matrix
M_RB = diag([m, m, Iz]);

% Added Mass Inertia Matrix
M_A = diag([-Xu, -Yv, -Nr]);

% Mass Inertia matrix
M = M_RB + M_A;

% Drag matrix
D = diag([Xc, Yc, Nc]);

% Rigid-Body Coriolis and Centripetal Matrix
C_RB = [0, 0, -m*v_r;
        0, 0, m*u_r;
        m*v_r, -m*u_r, 0];

% Hydrodynamic Coriolis and Centripetal Matrix
C_A = [0, 0, Yv*v_r;
       0, 0, -Xu*u_r;
       -Yv*v_r, Xu*u_r, 0];

% Coriolis and Centripetal Matrix
C = C_RB + C_A;

% Calculates acceleration in BODY reference frame (d/dt V)
V_dot = M \ (tau - (C * V_r) - (D * V_r));

end