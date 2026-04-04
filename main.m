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

%{
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

%}

%{

% Part b
tspan = [0,10]; % Simulated over 10 seconds

y0 = [0, 0, -20, 0, 5 * (pi/180), 0, 0, 0, 0, 0, 0, 0]';

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
%}

%{

% Part c

tspan = [0,10]; % Simulated over 10 seconds

y0 = [0, 0, -20, 0, 0, 0, 0, 0, 0, 0.1, 0, 0]';

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



%}

%{
% Part d

tspan = [0,10]; % Simulated over 10 seconds

y0 = [0, 0, -20, 0, 0, 0, 0, 0, 0, 0, 0.1, 0]';

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
%}




%% Part 3.4 Non Linear Model

%% 3.4 a
%{
% Part a
tspan = [0,10]; % Simulated over 10 seconds

%5 degrees in Roll

y0 = [0, 0, -20, 5 * (pi/180), 0, 0, 0, 0, 0, 0, 0, 0]';

[t,y] = ode45(@(t, y) QuadrotorEOMNonLinearClosedLoop(t, y, g, m, I, d, km, nu, mu, motor_forces), tspan, y0);

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
%}

%% 3.4 b
%{
% Part b
tspan = [0,10]; % Simulated over 10 seconds

%5 degrees in Pitch

y0 = [0, 0, -20, 0, 5 * (pi/180), 0, 0, 0, 0, 0, 0, 0]';

[t,y] = ode45(@(t, y) QuadrotorEOMNonLinearClosedLoop(t, y, g, m, I, d, km, nu, mu, motor_forces), tspan, y0);

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
%}

%% 3.4 c
%{
% Part c
tspan = [0,10]; % Simulated over 10 seconds

%0.1 rad/s roll

y0 = [0, 0, -20, 0, 0, 0, 0, 0, 0, 0.1, 0, 0]';

[t,y] = ode45(@(t, y) QuadrotorEOMNonLinearClosedLoop(t, y, g, m, I, d, km, nu, mu, motor_forces), tspan, y0);

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

%}

%% 3.4 d
%{
% Part d
tspan = [0,10]; % Simulated over 10 seconds

%0.1 rad/s pitch rate

y0 = [0, 0, -20, 0, 0, 0, 0, 0, 0, 0, 0.1, 0]';

[t,y] = ode45(@(t, y) QuadrotorEOMNonLinearClosedLoop(t, y, g, m, I, d, km, nu, mu, motor_forces), tspan, y0);

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

%}
%}

%% 3.5 



g = 9.81;

Ix = 5.8e-5;
Iy = 7.2e-5;

k3 = [linspace(0, 1e-5, 501), linspace(1e-5,1e-3,1000), linspace(1e-3, 1, 501)];
k3(501)  = [];
k3(1001) = [];

Eigenvalues_all = zeros(4, length(k3)); % store eigenvalues

for i = 1:length(k3)

    AclLinearizedLateral = [0, 1, 0, 0;
                            0, 0, g, 0;
                            0, 0, 0, 1;
                            0, -k3(i)/Ix, -40, -22];

    [V, D] = eig(AclLinearizedLateral);

    Eigenvalues_all(:, i) = diag(D); % store eigenvalues
end

sigma_max                = -1 / 1.25;
eig_real                 = real(Eigenvalues_all);
eig_im                   = imag(Eigenvalues_all);
eig_diff                 = eig_real - sigma_max;
eig_discard              = any( and(eig_real ~= 0, or(eig_diff > 0, eig_im ~= 0)), 1 );
eig_diff(:, eig_discard) = inf;
[~, eig_diff_min]        = min(abs(eig_diff), [], "all");
[~, eig_diff_min_col]    = ind2sub(size(Eigenvalues_all), eig_diff_min);
k3_min                   = k3(eig_diff_min_col);

% Root Locus Plot
colors = [
    linspace(0, 1, length(k3))', ...
    linspace(1, 0, length(k3))', ...
    zeros(length(k3), 1)
];

figure;
hold on;

for i = 1:length(k3)
    scatter(real(Eigenvalues_all(:,i)), imag(Eigenvalues_all(:,i)), 2, colors(i, :));
end

xline(-1 / 1.25, Color="red");

xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Lateral Eigenvalue Locus vs k3');
grid on;

%Linearized Longitudinal




