function [Fc,Gc] = VelocityReferenceFeedback(t, var)
% VelocityReferenceFeedback - Inner + outer loop controller with
%   feedforward reference commands for inertial translation.
%
% Inputs:
%   t   - current simulation time (s)
%   var - 12x1 state vector:
%         [u, v, w, p, q, r, x_E, y_E, z_E, phi, theta, psi]
%
% Outputs:
%   Fc  - 3x1 control force in body frame [Fx; Fy; Fz] (N)
%   Gc  - 3x1 control moment in body frame [L; M; N]   (N*m)

%% ── Parameters (hard-coded) ──────────────────────────────────────────────
m  = 0.068;    % mass (kg)
g  = 9.81;     % gravity (m/s^2)
Ix = 5.8e-5;   % roll  moment of inertia (kg*m^2)
Iy = 7.2e-5;   % pitch moment of inertia (kg*m^2)
Iz = 1.0e-4;   % yaw   moment of inertia (kg*m^2)

% state vector = [x, y, z, phi, theta, psi, u, v, w, p, q, r]
phi = var(4);   
theta = var(5);
psi = var(6);
u_b = var(7);
v_b = var(8);
w_b = var(9);
p = var(10);
q = var(11);   
r = var(12);    

% Move 1 m in 2 s then come to rest
t_end = 2; %s
dist = 1; %m

if t >= 0 && t < t_end % Lateral -> delta_vR active from t = [0, 0 + 2) sec
    V_ref = dist / t_end;  
    U_ref = 0;
elseif t >= 4 && t < 4+t_end % Longitudinal -> delta_uR active from t = [4, 4 + 2) sec
    V_ref = 0;
    U_ref = dist / t_end; 
end

%% ── Control Gains ─────────────────────────────────────────── !!!!!!
K1 = 22*Ix;      % derivative gain (roll rate / pitch rate)
K2 = 40*Ix;      % proportional gain (roll / pitch)
%K3 = ;    % outer loop: velocity error -> reference angle !!!!!!!!!

%% ------------

% Control Forces
Zc = -m*g;

% Control moments
Lc = -K2 * phi - K1 * p + K3 * (V_ref - v_b); % roll moment
Mc = -K2 * phi - K1 * p - K3 * (U_ref - u_b); % pitch moment
Nc = 0;

Fc = [0; 0; Zc];% body-z down convention
Gc = [Lc; Mc; Nc];% control moments

end