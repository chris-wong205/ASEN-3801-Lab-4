function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
% SOVANN BONINI - QuadrotorEOM_Linearized
%   Inputs:
    % var = state vector; 
    % g = gravitational constant; 
    % m = mass; 
%
%   Outputs: 
%   Fc - Control Forces
%   Gc - Control Moments

Xc = 0;
Yc = 0;
Zc = -1*m*g;

gain = 0.004; %Nm/(rad/sec)

angular_rate = var(10:12);

Fc = [Xc, Yc, Zc];
Gc = -1*gain*angular_rate;

end