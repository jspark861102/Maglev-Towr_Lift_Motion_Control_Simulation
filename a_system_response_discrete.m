clear all
close all
clc

%% parameters
omega = 0 : 0.5 : 2*pi*2500;
dt = 1/5000;

%%% PI Controller gain
ki = 500; %<1900
kp = 0.5;
%%%%%%%%%%%%%%%%%%%%%%%

%%% acc filter gain
tau = 1/(2*pi*100); %LPF
b = 2*pi*1; %HPF
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

%% acc sensor filter
% anaytic solution
Num1 = [dt   dt];
Den1 = [dt+2*tau, dt-2*tau];
sys1 = tf(Num1,Den1,dt);

[mag1, phase1, wout1] = bode(sys1, omega);
mag11(:,:) = mag1(1,:,:);
phase11(:,:) = phase1(1,:,:);

figure;
subplot(211)
semilogx(wout1/2/pi, 20*log10(abs(mag11)),'b','LineWidth', 2)
grid on
ylabel('magnitude (dB)')
xlim([0 1000])
ylim([-10 1])
title('acc sensor filter discrete')
set(gca,'fontsize', 14);

subplot(212)
semilogx(wout1/2/pi, phase11,'b','LineWidth', 2)
grid on
xlabel('frequency (Hz)')
ylabel('phase (deg)')
xlim([0 1000])
ylim([-100 10])
set(gca,'fontsize', 14);

%% current reference filter
% anaytic solution
% Num2 = [2   -2];
% Den2 = [w_current*dt +2, w_current*dt-2];
% sys2 = tf(Num2,Den2,dt);


a1 = (-8/dt^2 + 2*w_current)/(4/dt^2 + 4*zeta*w_current/dt + w_current^2);
a2 = (4/dt^2 - 4*zeta*w_current/dt + w_current^2)/(4/dt^2 + 4*zeta*w_current/dt + w_current^2);
b0 = (4/dt^2)/(4/dt^2 + 4*zeta*w_current/dt + w_current^2);
b1 = (-8/dt^2)/(4/dt^2 + 4*zeta*w_current/dt + w_current^2);
b2 = (4/dt^2)/(4/dt^2 + 4*zeta*w_current/dt + w_current^2);
c = (dt-2*tau_current) / (dt+2*tau_current);
d = dt/(dt+2*tau_current);

Num2 = [b0*d (b0+b1)*d (b1+b2)*d b2*d];
Den2 = [1 c+a1 a1*c+a2 a2*c];
sys2 = tf(Num2,Den2,dt);

[mag2, phase2, wout2] = bode(sys2, omega);
mag22(:,:) = mag2(1,:,:);
phase22(:,:) = phase2(1,:,:);

figure;
subplot(211)
semilogx(wout2/2/pi, 20*log10(abs(mag22)),'b','LineWidth', 2)
grid on
ylabel('magnitude (dB)')
xlim([0 1000])
ylim([-15 5])
title('current reference filter discrete')
set(gca,'fontsize', 14);

subplot(212)
semilogx(wout2/2/pi, phase22,'b','LineWidth', 2)
grid on
xlabel('frequency (Hz)')
ylabel('phase (deg)')
xlim([0 1000])
% ylim([-100 100])
set(gca,'fontsize', 14);


% filterDesigner solution
% Num2 = [ 0.0062    0.0062         0];
% Den2 = [ 1.0000   -0.9875         0];
% figure;freqz(Num2,Den2,10000)

%% current reference filter, notch filter
% as w0 << 1, parameters should be precise in 1e-7
Num3 = [1   -2*cos(w0)    1];
Den3 = [1   -2*r*cos(w0)  r*r];
sys3 = tf(Num3,Den3,dt);

[mag3, phase3, wout3] = bode(sys3, omega);
mag33(:,:) = mag3(1,:,:);
phase33(:,:) = phase3(1,:,:);

figure;
subplot(211)
semilogx(wout3/2/pi, 20*log10(abs(mag33)),'b','LineWidth', 2)
grid on
ylabel('magnitude (dB)')
xlim([0 1000])
% ylim([-10 1])
title('current reference notch filter discrete')
set(gca,'fontsize', 14);

subplot(212)
semilogx(wout3/2/pi, phase33,'b','LineWidth', 2)
grid on
xlabel('frequency (Hz)')
ylabel('phase (deg)')
xlim([0 1000])
% ylim([-100 100])
set(gca,'fontsize', 14);
