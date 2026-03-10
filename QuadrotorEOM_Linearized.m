function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
% SOVANN BONINI - QuadrotorEOM_Linearized
%   Inputs:
    %t = time span; 
    % var = state vector; 
    % g = gravitational constant; 
    % m = mass; 
    % I = 3x3 inertia matarix; 
    % deltaFc = Force deviations; 
    % deltaGc =  Moment deviations
%
%   Outputs: var_dot = derivative of the statevector
%
%   - Assigning each vector part of the state vector to its own vector
%   - Using the Linearized Quadrotor EOMs to generate a x_dot state vector

position = var(1:3); 
attitude = var(4:6);
velocity = var(7:9);
angular_rates = var(10:12);

position_dot = velocity;
attitude_dot = angular_rates;
velocity_dot = g * [-1*attitude(2), attitude(1), 0*attitude(3)]' + deltaFc * (1/m);
angular_rates_dot = [1/I(1,1) * deltaGc(2), 1/I(2,2) * deltaGc(2), 1/I(3,3) * deltaGc(3)]'; %=0

var_dot = [position_dot; attitude_dot; velocity_dot; angular_rates_dot];

end