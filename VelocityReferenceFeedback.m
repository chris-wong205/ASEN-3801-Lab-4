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

%This is the code associated with 3.6

%% ── Parameters (hard-coded) ──────────────────────────────────────────────
m  = 0.068;    % mass (kg)
g  = 9.81;     % gravity (m/s^2)
Ix = 5.8e-5;   % roll  moment of inertia (kg*m^2)
Iy = 7.2e-5;   % pitch moment of inertia (kg*m^2)
Iz = 1.0e-4;   % yaw   moment of inertia (kg*m^2)

% state vector extraction
phi = var(4);   
theta = var(5);
psi = var(6);
u_b = var(7);
v_b = var(8);
w_b = var(9);
p = var(10);
q = var(11);   
r = var(12);    

%% ── Reference Commands ───────────────────────────────────────────────────
if t <= 2
    %Longitudinal Case
    %V_ref=0;
    %U_ref=4.5;

    %Lateral Case
    V_ref=4.5;
    U_ref=0;

else 
    V_ref=0;
    U_ref=0;
end

%% ── Control Gains ────────────────────────────────────────────────────────
% Roll gains (scaled by Ix)
K1_roll = 22 * Ix;      % derivative gain (roll rate)
K2_roll = 40 * Ix;      % proportional gain (roll)

% Pitch gains (scaled by Iy for perfectly decoupled dynamics)
K1_pitch = 22 * Iy;     % derivative gain (pitch rate)
K2_pitch = 40 * Iy;     % proportional gain (pitch)

K3Longitudinal = -6.5e-5;
K3Lateral = 5.25e-5;           % outer loop: velocity error -> reference angle 

%% ── Control Forces & Moments ─────────────────────────────────────────────
% Control Forces
Zc = -m*g;
Fc = [0; 0; Zc]; % body-z down convention

% Control moments

Lc = -K2_roll * phi - K1_roll * p + K3Lateral * (V_ref - v_b); 

% FIX 2: Replaced 'phi' and 'p' with 'theta' and 'q'
Mc = -K2_pitch * theta - K1_pitch * q + K3Longitudinal * (U_ref - u_b); 

 Nc= -0.004* r;
Gc = [Lc; Mc; Nc]; % control moments

end