function [Fc, Gc] = InnerLoopFeedback(var)
% for a quadrotor in a hover trim state based on Problem 3.1 feedback gains.
%
% Inputs:
%   var : 12x1 State Vector
%       [x_E; y_E; z_E; u; v; w; phi; theta; psi; p; q; r]
%
% Outputs:
%   Fc : 3x1 Control Force Vector [Fx_c; Fy_c; Fz_c] in body frame
%   Gc : 3x1 Control Moment Vector [L_c; M_c; N_c] in body frame

%% 1. Quadrotor Physical Parameters 
   
    m = 0.068;            % Mass of quadrotor [kg] (Assume 1kg if not specified)
    g = 9.81;           % Gravity [m/s^2]
    Weight = m * g;     % Weight [N]
    
    Ix = 5.8e-5;          % Moment of inertia about x-axis [kg*m^2]
    Iy = 7.2e-5;          % Moment of inertia about y-axis [kg*m^2]
    Iz = 1e-4;          % Moment of inertia about z-axis [kg*m^2]

 %% 2. Define Feedback Control Gains % These were derived from a user inputed degree of one to dominate the other by a factor of 10.
    
    k1 = Ix * 22; % Roll Rate p
    k2 = Ix * 40; % Phi Gain
    k3 = Iy * 22; % Pitch rate q
    k4 = Iy * 40; % Theta Gain
   
    K_p     = k1/Ix;
    K_phi   = k2/Ix;
    K_q     = k3/Iy;
    K_theta = k4/Iy;
    
    
    
    % From Problem 2.3 (Spin portion - assuming a simple P-gain for yaw rate)

    K_r = 0.004; % Nm/rad/s Placeholder value!

    %% 3. Extract States from 12x1 Vector
    
    phi   = var(4);
    theta = var(5);
    psi   = var(6);  
    
    p = var(10);
    q = var(11);
    r = var(12);

    %% 4. Calculate Control Forces (Fc)
    % The control force in the body z-direction equals the weight (hover condition)
    % Forces in x and y are zero since attitude control maneuvers the thrust vector.
    Fx_c = 0;
    Fy_c = 0;
    Fz_c = -Weight; % Negative because body Z positive points down

    Fc = [Fx_c; Fy_c; Fz_c];

    %% 5. Calculate Control Moments (Gc)
    % Apply the control laws: u_norm = -K_angle*angle - K_rate*rate
    % Then convert normalized angular acceleration to physical torque: Torque = I * u_norm
    
    L_c = (- k1 * p -k2 * phi);
    M_c = (- k3 * q -k4 * theta);
    N_c = (-K_r * r); 

    Gc = [L_c; M_c; N_c];

end