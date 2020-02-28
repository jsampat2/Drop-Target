%% Squishy Robotics - Drop Targetting
clear all; clc; close all;
%%%% Assumptions
% Only a 3D plane is currently considered
% Robot is modeled as uniform rigid sphere
% Wind speed is considered variable and random in positive x direction

%% Initial Values
%%%% Known Data:
Cd = 0.5;
D = 21.5 * 0.0254; % m
A = pi * (D/2)^2; %Area of robot in m^2
m = 1.15; % kg
rho = 1.225; %kg/m^3
g = 9.81; %m/s^2

v_wind1 = rand(1); % Assumption
v_wind2 = rand(1);
k = (0.5) * (rho) * A * Cd;

%%%% Initial conditions
xhx0 = 0;
xv0 = 30.48; %100 ft in meters
xv_dot0 = 0;
xhx_dot0 = v_wind1; %when drone is hovering during drop
xhz0 = 0;
xhz_dot0 = v_wind2;
% xv_ddot0 = 0;
% xh_ddot0 = 0;

x0 = [xhx0; xhx_dot0; xhz0; xhz_dot0; xv0; xv_dot0];


%Terminal velocity calculator
v_t = sqrt((m * g)/k);  % should be 14.5m/s

%Time of Impact
%t_impact = (v_t/g) * inv(cosh(exp(xh0*g/(v_t)^2)));



%% Dynamics

%%%%%%%%%%%%%%%%%%%% FREE FALL BEFORE TERMINAL VELOCITY %%%%%%%%%%%%%%%%%

%%%% Initializing the fall dynamics
func1 = @(t,x) freefall(t,x,Cd, A, m, rho, g); % modeling only initial wind 
func2 = @(t,x) freefallwind(t,x,Cd, A, m, rho, g) % modeling constantly recurring wind
options = odeset('Events',@(t,x) event_before_vt(t,x,-v_t));

%%%% Initializing time and data vector
Tspan = [0 10] ;
t0 = 0 ; % Initial Time
t_vec = [] ; 
x1_vec = []; %constant wind
x2_vec = []; %dynamic wind


% Continuous Dynamics
[t,x1] = ode45(func1, t0+Tspan, x0, options) ;
[t,x2] = ode45(func2, t0+Tspan, x0, options) ;

% Save simulation data
t_vec = [t_vec; t] ;
x1_vec = [x1_vec; x1] ;
x2_vec = [x2_vec; x2] ;
       
% Reinitialize vector
x01 = x1_vec(end,:);
x02 = x2_vec(end,:);
t0 = t_vec(end);


%%%%%%%%%%%%%%%%%%%% FREE FALL AFTER TERMINAL VELOCITY %%%%%%%%%%%%%%%%%

func1 = @(t,x) terminalfall(t,x,Cd, A, m, rho, g, v_t);
func2 = @(t,x) terminalfallwind(t,x,Cd, A, m, rho, g, v_t);
options = odeset('Events',@(t,x) event_before_ground(t,x,xv0));

% Continuous Dynamics
[t,x1] = ode45(func1, t0+Tspan, x01, options) ;
[t,x2] = ode45(func2, t0+Tspan, x02, options) ;

% Save simulation data
t_vec = [t_vec; t] ;
x1_vec = [x1_vec; x1] ;
x2_vec = [x2_vec; x2] ;      

       



figure();
plot3(x1_vec(:,1),x1_vec(:,3),x1_vec(:,5));
hold on;
plot3(x2_vec(:,1),x2_vec(:,3),x2_vec(:,5));
hold off;
xlabel("Horizontal x position");
xlabel("Horizontal y position");
zlabel("Vertical z Position");
title("Trajectory");
%
disp('Only initial wind');
disp('X - cordinate:');disp(x1_vec(end,1));
disp('Y - cordinate:');disp(x1_vec(end,3));

disp('Repeating wind');
disp('X - cordinate:');disp(x2_vec(end,1));
disp('Y - cordinate:');disp(x2_vec(end,3));
%disp('X - cordinate:');disp(x_vec(end,5));disp('\n');