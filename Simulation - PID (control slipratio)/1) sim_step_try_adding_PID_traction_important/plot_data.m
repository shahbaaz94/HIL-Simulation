
% delete all variables
clear;

% close existing figure windows
close ALL;

% load array from file into matlab
% note that columns can be separated by space / tab or commas in matlab
A = load('sim.csv');

% file format
% time(s),x1,x2,x3,x4,x5,u1,u2,y1,y2,y3,y4

k = 1;
t  = A(:,k); k = k + 1;

x1 = A(:,k); k = k + 1;
x2 = A(:,k); k = k + 1;
x3 = A(:,k); k = k + 1;
x4 = A(:,k); k = k + 1;
x5 = A(:,k); k = k + 1;

u1 = A(:,k); k = k + 1;
u2 = A(:,k); k = k + 1;

y1 = A(:,k); k = k + 1;
y2 = A(:,k); k = k + 1;
y3 = A(:,k); k = k + 1;
y4 = A(:,k); k = k + 1;
a12 = A(:,k); k = k + 1;

k = 1;

% figure(k); k = k + 1;
% plot(t,x1);
% ylabel('x1, current, i (A)');
% xlabel('time (s)');
% 
% figure(k); k = k + 1;
% plot(t,x2);
% ylabel('x2, motor velocity, wm (rad/s)');
% xlabel('time (s)');
% 
% figure(k); k = k + 1;
% plot(t,x3);
% ylabel('x3, motor theta (rad)');
% xlabel('time (s)');
% 
% figure(k); k = k + 1;
% plot(t,x4);
% ylabel('x4, forward velocity v (m/s)');
% xlabel('time (s)');

figure(k); k = k + 1;
plot(t,x5);
ylabel('x5, forward position x (m)');
xlabel('time (s)');

figure(k); k = k + 1;
plot(t,u1);
ylabel('u1, motor voltage V(t)');
xlabel('time (s)');
% 
% figure(k); k = k + 1;
% plot(t,u2);
% ylabel('u2, disturbance torque Td(t)');
% xlabel('time (s)');
% 
% figure(k); k = k + 1;
% plot(t,y1);
% ylabel('y1, wb, back wheel velocity (rad/s)');
% xlabel('time (s)');
% 
% figure(k); k = k + 1;
% plot(t,y2);
% ylabel('y2, wf, front wheel velocity (rad/s) ');
% xlabel('time (s)');

figure(k); k = k + 1;
plot(t,y3);
ylabel('y3, S, slip ratio');
xlabel('time (s)');

figure(k); k = k + 1;
plot(t,y4);
ylabel('wheel velocity (rad/s)');
xlabel('time (s)');
ylabel('y4, mu, tire friction coefficient');
xlabel('time (s)');

% plot back wheel and forward wheel velocities together

figure(k); k = k + 1;
%frontwheel = stepinfo(y1,t)
backwheel = stepinfo(y2,t)
plot(t,y1,t,y2,'r');
legend('back wheel velocity wb','forward wheel velocity wf','Location','northeast');

figure(k); k = k + 1;
plot(t,a12);
ylabel('sum of acceleration of front and back wheel');
xlabel('time (s)');

