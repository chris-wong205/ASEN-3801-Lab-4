clc;
clear all; 
close all;

data = load("RSdata_nocontrol.mat");
%% Task 2

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

motor_forces = [m*g/4;m*g/4;m*g/4;m*g/4];

% Intial conditions that must be passed in for 2.1 & 2.2

y0 = [0,0,0,0,0,0,0,0,0,0.1,0,0]';
deltaFc = [0 0 0]';
deltaGc = [0 0 0]';

tspan = [0,10];

%% Inital State Vector
 
[t,y] = ode45(@(t, y) QuadrotorEOM(t, y, g, m, I, d, km, nu, mu, motor_forces),tspan,y0);
[t2,y2] = ode45(@(t, y) QuadrotorEOM_Linearized(t, y, g, m, I, deltaFc, deltaGc),tspan,y0);

time = t;
aircraft_state_array = y;
Zc = (-m*g);
control_input_array = [Zc;0;0;0];
fig = [1:6];
col = 'b-';

PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col);

time = t2;
aircraft_state_array = y2;
Zc = (-m*g);
control_input_array = [deltaFc(3); deltaGc];
fig = [1:6];
col = 'r-';

PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col);

[Fc, Gc] = RotationDerivativeFeedback(y0, m, g);
motor_forces = ComputeMotorForces(Fc, Gc, d, km);
[t3,y3] = ode45(@(t,y) QuadrotorEOMwithRateFeedback(t, y, g, m, I, nu, mu), tspan, y0);

time = t3;
aircraft_state_array = y3;
Zc = (-1*(motor_forces(1)+motor_forces(2)+motor_forces(3)+motor_forces(4)));
control_input_array = [Zc;0;0;0];
fig = [1:6];
col = 'm-';

PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col);
%saveAllOpenFigures();
