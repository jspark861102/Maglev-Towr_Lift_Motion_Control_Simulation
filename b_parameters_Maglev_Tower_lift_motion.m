clear all
close all
clc

%% simulation parameters ==========================================
%1 : 1st order HPF
%2 : 2nd order HPF 
%3 : 2nd order HPF + 1st order LPF
%4 : digital notch filter
select_current_filter = 3; 

isdiscrete = 0; % 0:continuous, 1:discrete
isnoise = 1; % 0:without noise, 1:with noise 

%% system parameters ==========================================
fs = 5e3;                               % Sampling freq.: 5kHz
dt = 1/fs;
Ns = 5e3;                               % Number of measurements
t = 0:dt:(Ns-1)*dt;                     % Measurement time

%%% Carrier Parameters
h = 327.07 * 1e-3;
w = 219.77 * 1e-3;
d = 129.47 * 1e-3;

ha = 373.57 * 1e-3;
wa = 140.52 * 1e-3;
da = 239.47 * 1e-3;

M = 302.667;    
g = 9.81;

Ix_cg =41.394;
Iy_cg =30.050;
Iz_cg =14.165;

Ix = Ix_cg;
Iy = Iy_cg + M*da^2;
Iz = Iz_cg + M*da^2;

%%% magnet parmeters
Area = 120*100e-6; %m^2
N = 270;           %turn
R = 0.5;           %ohm
c0 = 0.001;        %m
ki_current = 1;
kc_current = 1;
%%%%%%%%%%%%%%%%%%%%%%%

%%% PI Controller gain
ki = 500; %<1900
kp = 0.5;
%%%%%%%%%%%%%%%%%%%%%%%

%%% acc filter gain
tau = 1/(2*pi*100); %LPF
b = 0; %HPF
%%%%%%%%%%%%%%%%%%%%%%%

%%% current filter gain
%high pass filter
w_current = 2*pi*5;
tau_current = 1/(2*pi*100);
zeta = 0.707;

%IIR notch filter (50Hz)
w0 = 2*pi*50/5000;
r = 0.99;
B0 =  1;
B1 = -2*cos(w0);
B2 =  1;
A1 =  -2*r*cos(w0);
A2 =  r*r;
%%%%%%%%%%%%%%%%%%%%%%%

%%% cutoff frequency print
fc_controller_hz = ki/(1+kp)/2/pi;
zero_controller_hz = ki/kp/2/pi;
fc_acc_filter_low_hz = 1/tau/2/pi;
fc_acc_filter_high_hz = b/2/pi;
fc_current_filter_hz = w_current/2/pi;
fprintf('[fc_controller, zero_controller, fc_acc_low, fc_acc_high, fc_current] = [%0.3fhz, %0.3fhz, %0.3fhz, %0.3fhz, %0.3fhz]\n',fc_controller_hz, zero_controller_hz, fc_acc_filter_low_hz, fc_acc_filter_high_hz, fc_current_filter_hz);
%%%%%%%%%%%%%%%%%%%%%%%

%%% reference 
droll_ref_mag = 0.001;    
droll_ref_frequency = 2*pi*0.1;
dpitch_ref_mag = 0.001;    
dpitch_ref_frequency = 2*pi*0.5;
dyaw_ref_mag = 0.001;    
dyaw_ref_frequency = 2*pi*1;
dx_ref_mag = 0.001;    
dx_ref_frequency = 2*pi*5;
dy_ref_mag = 0.001;    
dy_ref_frequency = 2*pi*10;
%%%%%%%%%%%%%%%%%%%%%%%

%%% initial conditions 
initial_conditions
%%%%%%%%%%%%%%%%%%%%%%%

%% System Matrix =================================================
%state = [droll, dpitch, dyaw, dx, dy]
%input = [F1x, F2x, F3x, F4x, F1y, F2y, F3y, F4y]
% dX = A*X + B*U = Bu*(Bv*U) = Bu*ddqd

A = [zeros(5,5)];
Ki = diag([ki_current, ki_current, ki_current, ki_current, ki_current]);
B = Ki * [  0     0     0     0     -h/Ix  h/Ix  h/Ix  -h/Ix;
            h/Iy -h/Iy  h/Iy -h/Iy   0     0     0      0;
            w/Iz  w/Iz -w/Iz -w/Iz  -d/Iz -d/Iz  d/Iz   d/Iz;    
            1/M   1/M   1/M   1/M    0     0     0      0;
            0     0     0     0      1/M   1/M  -1/M   -1/M];

Bu = B;
Bv = eye(5,5);
pBu = pinv(Bu);

% discrete
Ad = A*dt + eye(size(A));
Bd = B*dt;

Bdu = Bd;
Bdv = eye(5,5)*dt;
pBdu = pinv(Bdu);

%% PI Controller gain for each DOF ==========================================
ki_roll =  ki;
ki_pitch = ki;
ki_yaw =   ki;
ki_x =     ki;
ki_y =     ki;

kp_roll =  kp;
kp_pitch = kp;
kp_yaw =   kp;
kp_x =     kp;
kp_y =     kp;

