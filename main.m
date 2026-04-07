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
%Lateral Locus Plot and Data
g = 9.81;

Ix = 5.8e-5;
Iy = 7.2e-5;

lateral_k3       = [linspace(0, 1e-5, 501), linspace(1e-5,1e-3,1000), linspace(1e-3, 1, 501)]';
lateral_k3(501)  = [];
lateral_k3(1001) = [];
lateral_k3_len   = length(lateral_k3);

AclLinearizedLateral = @(k3) [0, 1, 0, 0;
                              0, 0, g, 0;
                              0, 0, 0, 1;
                              0, -k3/Ix, -40, -22];
[lateral_k3_min, lateral_eigenvalues] = k3_min(AclLinearizedLateral, length(AclLinearizedLateral(0)), lateral_k3);

% Root Locus Plot
colors = [ 
    0, 0, 0, 1;
    1, 1, 0, 1;
];

lateral_k3_transformed = gradient_transform(lateral_k3, 0.2);

lateral_colors = [
    interp1(colors(:, 1), colors(:, 2), lateral_k3_transformed), ... 
    interp1(colors(:, 1), colors(:, 3), lateral_k3_transformed), ...
    interp1(colors(:, 1), colors(:, 4), lateral_k3_transformed)
];

figure;
hold on;

colormap(lateral_colors);

for i = 1 : lateral_k3_len
    scatter(real(lateral_eigenvalues(:,i)), imag(lateral_eigenvalues(:,i)), 2, lateral_colors(i, :));
end

cb = colorbar();
cb.Label.String = "k3";

xline(-1 / 1.25, Color="red");

xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Lateral Eigenvalue Locus');
grid on;

% Logitudinal Locus Plot and Data
longitudinal_k3 = [linspace(-1, -1e-3, 501), linspace(-1e-3, -1e-5, 1000), linspace(-1e-5, 0, 501)]';
longitudinal_k3(501) = [];
longitudinal_k3(1001) = [];
longitudinal_k3_len = length(longitudinal_k3);

AclLinearizedLongitudinal = @(k3) [0, 1,       0,   0;
                                   0, 0,      -g,   0;
                                   0, 0,       0,   1;
                                   0, -k3/Iy, -40, -22];
[longitudinal_k3_min, longitudinal_eigenvalues] = k3_min(AclLinearizedLongitudinal, length(AclLinearizedLongitudinal(0)), longitudinal_k3);

% Root Locus Plot
longitudinal_k3_transformed = gradient_transform(longitudinal_k3, 0.2);

longitudinal_colors = [
    interp1(colors(:, 1), colors(:, 2), longitudinal_k3_transformed), ...
    interp1(colors(:, 1), colors(:, 3), longitudinal_k3_transformed), ...
    interp1(colors(:, 1), colors(:, 4), longitudinal_k3_transformed)
];

figure;
hold on;

colormap(longitudinal_colors);

for i = 1 : longitudinal_k3_len
    scatter(real(longitudinal_eigenvalues(:,i)), imag(longitudinal_eigenvalues(:,i)), 2, longitudinal_colors(i, :));
end

cb = colorbar();
cb.Label.String = "k3";
clim([min(longitudinal_k3), max(longitudinal_k3)]);

xline(-1 / 1.25, Color="red");

xlabel('Real Axis');
ylabel('Imaginary Axis');
title('Longitudinal Eigenvalue Locus');
grid on;

%% 3.7 Simulation of Varried Control Lateral and Logitudinal

%To change between longitudinal and lateral simply change the values of Vx
%and Ux to the correct values

% tspan = [0,10]; % Simulated over 10 seconds
%
%
% y0 = [0, 0, -20, 0, 0, 0, 0, 0, 0, 0, 0, 0]';
% func = @(t, y) QuadrotorEOMNonLinearOpenLoop(t, y, g, m, I, d, km, nu, mu, motor_forces);
% [t,y] = ode45(func, tspan, y0);
%
% % Preallocate
%
% % 1. Evaluate the EoM over all time steps. 
%
% [~, Control_cells] = cellfun(@(t_val, y_row) func(t_val, y_row'), num2cell(t), num2cell(y, 2), 'UniformOutput', 0);
%
% % 2. Convert the cell array directly into your N x 4 matrix.
%
% control_input_array = cell2mat(Control_cells); 
%
% % Optional: If you still want the individual column vectors for plotting
% Z_c = control_input_array(:, 1);
% L_c = control_input_array(:, 2);
% M_c = control_input_array(:, 3);
% N_c = control_input_array(:, 4);
%
% time = t;
% aircraft_state_array = y;
%
% fig = [1:6];
% col = 'b-';
%
% PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
% saveAllOpenFigures()
