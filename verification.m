%% Final Project Verification
% Damped & Forced Vehicle Bounce & Pitch

% Clear previous data & command window
clear all
clc

% Define parameters & display
m = 1926.4; 
J = 2822.4; 
k_f = 16680.5;
k_r = 23204.3; 
l1 = 0.76;  
l2 = 0.79;  
c_f = 125.4;  
c_r = 125.3; 
% Initial conditions and time
IC = [0 0 0 0]; % [x x' theta theta'] 
time = [0 5]; % [s]
fprintf('The input parameters are:\n m = %5.1f kg\n J = %5.1f kg*m^2\n',...
    m, J)
fprintf(' kf = %5.1f N/m\n kr = %5.1f N/m\n L1 = %3.2f m\n L2 = %3.2f m\n',...
    k_f, k_r, l1, l2)
fprintf(' cf = %3.1f N*s/m\n cr = %3.1f N*s/m\n x(0) = %3.1f m\n',c_f,...
    c_r, IC(1))
fprintf(' timespan = 0-%g s\n',time(2))

% Determine radial frequency & y parameters
vel = 10; % [m/s]
L = 0.5; % [m]
A = 0.025; % [m]
h = A/2; % [m]
radFreq = (2*pi*vel)/L; % [rad/s]
fprintf('\n')
fprintf('The velocity of the vehicle is: %g m/s\n', vel)

% Solve 4 coupled ODE's
[t,sltn]=ode45(@(t,y)coupledEqtns(t,y,m,J,k_f,k_r,l1,l2,c_f,c_r,...
    radFreq,A,L),time,IC);

% Plot results
plot(t,sltn(:,1),t,sltn(:,3)) % Plot x and theta
title('Displacement and Pitch vs. Time')
xlabel('Time [s]')
ylabel('Amplitude')
legend('Displacement [m]','Pitch [rad]','location','NorthEast')

% Determine maximum values
maximumPitch = max(sltn(:,3));
maximumDisplacement = max(sltn(:,1));

% Display results
fprintf('The distance between bumps is: %3.2f m, and the height is: %3.3f m\n'...
    ,L,h)
fprintf('The maximum pitch of the vehicle is: %5.4E rad\n', maximumPitch)
fprintf('The maximum bounce of the vehicle is: %5.4E m\n',...
    maximumDisplacement)