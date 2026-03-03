function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)
% PlotAircraftSim
% Plots aircraft states over time

% State Vector Extraction
x = aircraft_state_array(:,1);
y = aircraft_state_array(:,2);
z = aircraft_state_array(:,3);
phi   = aircraft_state_array(:,4);
theta = aircraft_state_array(:,5);
psi   = aircraft_state_array(:,6);
u = aircraft_state_array(:,7);
v = aircraft_state_array(:,8);
w = aircraft_state_array(:,9);
p = aircraft_state_array(:,10);
q = aircraft_state_array(:,11);
r = aircraft_state_array(:,12);

%% Position Plot
figure(fig(1));

subplot(3,1,1)
plot(time,x,col,'LineWidth',1.5)
hold on
plot(time(1),x(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),x(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('x (m)')
title('X Position')
grid on

subplot(3,1,2)
plot(time,y,col,'LineWidth',1.5)
hold on
plot(time(1),y(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),y(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('y (m)')
title('Y Position')
grid on

subplot(3,1,3)
plot(time,z,col,'LineWidth',1.5)
hold on
plot(time(1),z(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),z(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('z (m)')
title('Z Position')
grid on

%% Euler Angle Plots
figure(fig(2));

subplot(3,1,1)
plot(time,phi,col,'LineWidth',1.5)
hold on
plot(time(1),phi(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),phi(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('\phi (rad)')
title('Roll')
grid on

subplot(3,1,2)
plot(time,theta,col,'LineWidth',1.5)
hold on
plot(time(1),theta(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),theta(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('\theta (rad)')
title('Pitch')
grid on

subplot(3,1,3)
plot(time,psi,col,'LineWidth',1.5)
hold on
plot(time(1),psi(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),psi(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('\psi (rad)')
title('Yaw')
grid on

%% Velocity Plot
figure(fig(3));

subplot(3,1,1)
plot(time,u,col,'LineWidth',1.5)
hold on
plot(time(1),u(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),u(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('u (m/s)')
title('Body X Velocity')
grid on

subplot(3,1,2)
plot(time,v,col,'LineWidth',1.5)
hold on
plot(time(1),v(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),v(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('v (m/s)')
title('Body Y Velocity')
grid on

subplot(3,1,3)
plot(time,w,col,'LineWidth',1.5)
hold on
plot(time(1),w(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),w(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('w (m/s)')
title('Body Z Velocity')
grid on

%% Euler Angle Rate Plots
figure(fig(4));

subplot(3,1,1)
plot(time,p,col,'LineWidth',1.5)
hold on
plot(time(1),p(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),p(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('p (rad/s)')
title('Roll Rate')
grid on

subplot(3,1,2)
plot(time,q,col,'LineWidth',1.5)
hold on
plot(time(1),q(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),q(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('q (rad/s)')
title('Pitch Rate')
grid on

subplot(3,1,3)
plot(time,r,col,'LineWidth',1.5)
hold on
plot(time(1),r(1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),r(end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('r (rad/s)')
title('Yaw Rate')
grid on

%% Control Input Plot
figure(fig(5));

subplot(4,1,1)
plot(time, control_input_array(1,:), col, 'LineWidth', 1.5)
hold on
plot(time(1),control_input_array(1,1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),control_input_array(1,end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('Zc')
title('Control Input Zc')
grid on

subplot(4,1,2)
plot(time, control_input_array(2,:), col, 'LineWidth', 1.5)
hold on
plot(time(1),control_input_array(2,1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),control_input_array(2,end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('Lc')
title('Control Input Lc')
grid on

subplot(4,1,3)
plot(time, control_input_array(3,:), col, 'LineWidth', 1.5)
hold on
plot(time(1),control_input_array(3,1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),control_input_array(3,end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('Mc')
title('Control Input Mc')
grid on

subplot(4,1,4)
plot(time, control_input_array(4,:), col, 'LineWidth', 1.5)
hold on
plot(time(1),control_input_array(4,1),'ro','MarkerFaceColor','r','MarkerSize',7)
plot(time(end),control_input_array(4,end),'go','MarkerFaceColor','g','MarkerSize',7)
xlabel('Time (s)')
ylabel('Nc')
title('Control Input Nc')
grid on

%% 3D Path of Aircraft
figure(fig(6));
plot3(x, y, z, col, 'LineWidth', 1.5);
hold on
plot3(x(1),y(1),z(1),'ro','MarkerFaceColor','r','MarkerSize',8)
plot3(x(end),y(end),z(end),'go','MarkerFaceColor','g','MarkerSize',8)
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Path of Aircraft');
grid on;
view(3);

end