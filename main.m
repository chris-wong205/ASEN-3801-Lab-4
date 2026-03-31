clc;
clear all; 
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

motor_forces = [m*g/4;m*g/4;m*g/4;m*g/4];

% Intial conditions that must be passed in for 1.2 and 1.3

 %y0 = [0,0,-10,0,0,0,0,0,0,0,0,0]';

%% Intial conditions that must be passed in for 1.4


phi0 = asin(0.3676/g); % This number was derived by determing the aero force for the IC of 5 and dividing by mg

vE = 5;
v0 = vE * cos(phi0);
w0 = - vE * sin(phi0);
% Define the initial conditions for the ODE
y0 = [0, 0, -20, phi0, 0, 0, 0, v0, w0, 0, 0, 0]'; %Starts the AC 20 meters up in the air


f = (m*g/cos(phi0))/4;

motor_forces = [f,f,f,f]';


tspan = [0,10];
%{
%% Inital State Vector

 
[t,y] = ode45(@(t, y) QuadrotorEOM(t, y, g, m, I, d, km, nu, mu, motor_forces),tspan,y0);

time = t;
aircraft_state_array = y;

control_input_array = [0;0;0;0];
fig = [1:6];
col = 'b-';

PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
saveAllOpenFigures()

%}
%% Task 3.3


% Part a

tspan = [0,10]; % Simulated over 10 seconds

y0 = [0, 0, -20, 5 * (pi/180), 0, 0, 0, 0, 0, 0, 0, 0]';

[t,y] = ode45(@(t, y) QuadrotorEOMClosedLoop(t, y, g, m, I), tspan, y0);

% Preallocate
Z_c = zeros(length(t),1);
L_c = zeros(length(t),1);
M_c = zeros(length(t),1);
N_c = zeros(length(t),1);

for i = 1:length(t)
    [~, Control] = QuadrotorEOMClosedLoop(t(i), y(i,:)', g, m, I);
    
    Z_c(i) = Control.Z;
    L_c(i) = Control.L;
    M_c(i) = Control.M;
    N_c(i) = Control.N;
end


control_input_array = [Z_c, L_c, M_c, N_c];

time = t;
aircraft_state_array = y;

fig = [1:6];
col = 'b-';

PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
saveAllOpenFigures()

close all;

% Part b
tspan = [0,10]; % Simulated over 10 seconds

y0 = [0, 0, -20, 0,5 * (pi/180), 0, 0, 0, 0, 0, 0, 0]';

[t,y] = ode45(@(t, y) QuadrotorEOMClosedLoop(t, y, g, m, I), tspan, y0);

% Preallocate
Z_c = zeros(length(t),1);
L_c = zeros(length(t),1);
M_c = zeros(length(t),1);
N_c = zeros(length(t),1);

for i = 1:length(t)
    [~, Control] = QuadrotorEOMClosedLoop(t(i), y(i,:)', g, m, I);
    
    Z_c(i) = Control.Z;
    L_c(i) = Control.L;
    M_c(i) = Control.M;
    N_c(i) = Control.N;
end


control_input_array = [Z_c, L_c, M_c, N_c];

time = t;
aircraft_state_array = y;

fig = [1:6];
col = 'b-';

PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
saveAllOpenFigures()



% Part c

% Part d