%David Robie
%AE 3713
%Homework 9a
%Modeling

clear all;
clc;

duration = 40;
theta_0 = -70;

%[t,y] = ode45(@pcontroller,[0 duration],[theta_0; 0]);
%[t,y] = ode45(@pdcontroller,[0 duration],[theta_0; 0]);
[t,y] = ode45(@pidcontroller,[0 duration],[theta_0; 0]);

%Array=csvread('PControl.csv');
%col1 = Array(:, 1);
%col2 = Array(:, 2);
%plot(col1,col2)
%hold on

%Array=csvread('PControl.csv');
%col1 = Array(1:541, 3);
%col2 = Array(1:541, 4);
%plot(col1,col2)
%hold on

%Array=csvread('PControl.csv');
%col1 = Array(1:640, 5);
%col2 = Array(1:640, 6);
%plot(col1,col2)
%hold on

%Array=csvread('PDControl.csv');
%col1 = Array(:, 2);
%col2 = Array(:, 3);
%plot(col1,col2)
%hold on

Array=csvread('PIDControl.csv');
col1 = Array(:, 2);
col2 = Array(:, 3);
plot(col1,col2)
hold on

%plot(t, y(:,1), '-',t, y(:,2), '-')
plot(t, y(:,1), '-')
%title('P Gain = 1 Modeled vs Measured');
%title('P Gain = 1, D gain = 0.15 Modeled vs Measured');
title('P Gain = 1, I Gain = 0.25, D gain = 0.15 Modeled vs Measured');
xlabel('Time (Seconds)'); 
ylabel('Pitch (Degrees)'); 
legend('\theta Measured','\theta Modeled');
%legend('\theta Modeled');
hold off

function dydt = pcontroller(t,y)
C1 = 17; 
C2 = -13732;
C3 = 1.381;
Kp = 1; % This is the proportional gain you want to test with
thetaDesired=0;
u = -Kp*(thetaDesired-y(1));
dydt = [y(2); -C1*y(2)+C2*cos(y(1)*pi/180)+C3*u^2];
end

function dydt = pdcontroller(t,y)
C1 = 17; 
C2 = -13732;
C3 = 1.381;
Kp = 1; % This is the proportional gain you want to test with
Kd = 0.15; % This is the derivative gain you want to test with
thetaDesired=0;
thetaDotDesired=0;
u = -(Kp*y(1) + Kd*y(2));
%u = -(Kp*(thetaDesired-y(1)) + Kd*(thetaDotDesired-y(2)));
dydt = [y(2); -C1*y(2)+C2*cos(y(1)*pi/180)+C3*u^2];
end

function dydt = pidcontroller(t,y)
C1 = 17; 
C2 = -13732;
C3 = 1.381;
Kp = 1; % This is the proportional gain you want to test with
Ki = 0.15; % This is the integral gain you want to test with
Kd = 0.25; % This is the derivative gain you want to test with
thetaDesired=0;
u = -(Kp*y(1) + Ki*y(1)*t + Kd*y(2));
%u = -(Kp*(thetaDesired-y(1)) + Ki*(thetaDesired-y(1))*t + Kd*y(2));
dydt = [y(2); -C1*y(2)+C2*cos(y(1)*pi/180)+C3*u^2];
end
