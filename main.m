clc;
clear; 
close all;

data = load("RSdata_nocontrol.mat");
%% Task 1

% Define necessary data components for ODE 45
m = 0.068; %kg
t = data.rt_estim.time; % Loads in the time vector f data
y = data.rt_estim.signals.values;% Loads in the State Vector of Data
g = 9.81; % m/s

I = [5.8e-5,0,0;
    0,7.2e-5,0
    0,0,1e-4]; %kg m^2
       
km = 0.0024;% N*m/N
d = 0.06; % Meters

nu = 1e-3; % N/(m/s)^2 Aerodnymaic force coefficient
mu = 2e-6; % N*m/(rad/s)^2 Aerodynamic moment coef

%Define motor forces
f1 = data.rt_motor.signals.values(:,1);
f2 = data.rt_motor.signals.values(:,2);
f3 = data.rt_motor.signals.values(:,3);
f4 = data.rt_motor.signals.values(:,4);

%% Intial conditions that must be passed in for 1.3
% motor_forces = [m*g/4;m*g/4;m*g/4;m*g/4];
% y0 = [0,0,-10,0,0,0,0,0,0,0,0,0]';

%% Intial conditions that must be passed in for 1.4
V_a = 5;
phi = atan2(V_a^2 * nu, m * g);

R = rotation_matrix([pi / 2, -phi, 0]);
% R = rotation_matrix([0, 0, phi]);
v0 = R * [0; 5; 0];

motor_forces = ( (m * g) / (4 * cos(phi)) ) * ones(4, 1);

% Define the initial conditions for the ODE
tspan = [0,10];
% y0 = [
%     zeros(3, 1);
%     phi;
%     0;
%     0;
%     v0;
%     zeros(3, 1);
% ]';

y0 = [
    zeros(3, 1);
    0;
    -phi;
    pi / 2;
    v0;
    zeros(3, 1);
]';

fig = 1 : 6;
col = 'b-';
control_input_array = [0; 0; 0; 0];

[t, state] = ode45(@(t, y) QuadrotorEOM(t, y, g, m, I, d, km, nu, mu, motor_forces), tspan, y0);
PlotAircraftSim(t, state, control_input_array, fig, col)
saveAllOpenFigures()
