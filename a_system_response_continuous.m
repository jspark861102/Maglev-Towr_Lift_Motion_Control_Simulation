clear all
close all
clc

%% parameters
omega = 0 : 0.5 : 2*pi*1000;

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

%% components tranfer function
%controller
Cm = tf([kp ki],[1 0]);

%acc sensor filter
Co_LPF = tf([1],[tau 1]);
Co_HPF = tf([1 0],[1 b]);
Co_band = Co_LPF*Co_HPF;

%current reference filter
Fi_1st = tf([1 0],[1 w_current]);
Fi_2nd = tf([1 0 0],[1 2*zeta*w_current w_current^2]);
Fi_band = Fi_2nd * tf([1],[tau_current 1]);

%current reference notch filter
Fi_notch = tf([B0 B1 B2],[1 A1 A2], 1/5000);
% figure;bode(Fi_notch)
%% system transfer function
%openloop
sys_op = tf([1], [1 0] );

%closedloop with controller
sys_Cm = tf([kp ki], [1+kp ki]);

% system w/ controller & band pass sensor filter 
sys_Cm_bandCo = (Cm)/(1+Cm*Co_band);
sys_Cm_bandCo_analytic = tf([tau*kp, ((b*tau + 1)*kp + tau*ki), ((b*tau + 1)*ki + b*kp), + b*ki], [tau, (b*tau+1)+kp, b+ki 0]);
sys_Cm_bandCo_check = sys_Cm_bandCo/sys_Cm_bandCo_analytic;

% system w/ controller & low pass sensor filter 
sys_Cm_LPFCo = (Cm)/(1+Cm*Co_LPF);
sys_Cm_LPFCo_analytic = tf([tau*kp, ((b*tau + 1)*kp + tau*ki), ((b*tau + 1)*ki + b*kp), + b*ki], [tau, (b*tau+1)+kp, b+ki 0]);
sys_Cm_LPFCo_check = sys_Cm_LPFCo/sys_Cm_LPFCo_analytic;

% system w/ controller & low pass sensor filter & 1st order current filter
sys_1stFi_Cm_LPFCo = (Fi_1st*Cm)/(1+Fi_1st*Cm*Co_LPF);
sys_1stFi_Cm_LPFCo_analytic = tf([tau*kp, kp + tau*ki, ki], [tau, 1+tau*w_current+kp, w_current+ki]);
sys_1stFi_Cm_LPFCo_check = sys_1stFi_Cm_LPFCo/sys_1stFi_Cm_LPFCo_analytic;

% system w/ controller & low pass sensor filter & 2nd order current filter
sys_2ndFi_Cm_LPFCo = (Fi_2nd*Cm)/(1+Fi_2nd*Cm*Co_LPF);

% system w/ controller & low pass sensor filter & 2nd order current band filter
sys_bandFi_Cm_LPFCo = (Fi_band*Cm)/(1+Fi_band*Cm*Co_LPF);

%% bode data
%open loop
[mag_op, phase_op, wout_op] = bode(sys_op, omega);
mag_op1(:,:) = mag_op(1,:,:);
phase_op1(:,:) = phase_op(1,:,:);

%acc sensor
[mag_Co_band, phase_Co_band, wout_Co_band] = bode(Co_band, omega);
mag_Co_band1(:,:) = mag_Co_band(1,:,:);
phase_Co_band1(:,:) = phase_Co_band(1,:,:);

%current filter
[mag_Fi_band, phase_Fi_band, wout_Fi_band] = bode(Fi_band, omega);
mag_Fi_band1(:,:) = mag_Fi_band(1,:,:);
phase_Fi_band1(:,:) = phase_Fi_band(1,:,:);

[mag_Fi_2nd, phase_Fi_2nd, wout_Fi_2nd] = bode(Fi_2nd, omega);
mag_Fi_2nd1(:,:) = mag_Fi_2nd(1,:,:);
phase_Fi_2nd1(:,:) = phase_Fi_2nd(1,:,:);

%w/ controller
[mag_Cm, phase_Cm, wout_Cm] = bode(sys_Cm, omega);
mag_Cm1(:,:) = mag_Cm(1,:,:);
phase_Cm1(:,:) = phase_Cm(1,:,:);

%wi/ controller & sensor filter
[mag_Cm_bandCo, phase_Cm_bandCo, wout_Cm_bandCo] = bode(sys_Cm_bandCo, omega);
mag_Cm_bandCo1(:,:) = mag_Cm_bandCo(1,:,:);
phase_Cm_bandCo1(:,:) = phase_Cm_bandCo(1,:,:);

[mag_Cm_LPFCo, phase_Cm_LPFCo, wout_Cm_LPFCo] = bode(sys_Cm_LPFCo, omega);
mag_Cm_LPFCo1(:,:) = mag_Cm_LPFCo(1,:,:);
phase_Cm_LPFCo1(:,:) = phase_Cm_LPFCo(1,:,:);

%wi/ controller & sensor filter & current filter
[mag_1stFi_Cm_LPFCo, phase_1stFi_Cm_LPFCo, wout_1stFi_Cm_LPFCo] = bode(sys_1stFi_Cm_LPFCo, omega);
mag_1stFi_Cm_LPFCo1(:,:) = mag_1stFi_Cm_LPFCo(1,:,:);
phase_1stFi_Cm_LPFCo1(:,:) = phase_1stFi_Cm_LPFCo(1,:,:);

[mag_2ndFi_Cm_LPFCo, phase_2ndFi_Cm_LPFCo, wout_2ndFi_Cm_LPFCo] = bode(sys_2ndFi_Cm_LPFCo, omega);
mag_2ndFi_Cm_LPFCo1(:,:) = mag_2ndFi_Cm_LPFCo(1,:,:);
phase_2ndFi_Cm_LPFCo1(:,:) = phase_2ndFi_Cm_LPFCo(1,:,:);

[mag_bandFi_Cm_LPFCo, phase_bandFi_Cm_LPFCo, wout_bandFi_Cm_LPFCo] = bode(sys_bandFi_Cm_LPFCo, omega);
mag_bandFi_Cm_LPFCo1(:,:) = mag_bandFi_Cm_LPFCo(1,:,:);
phase_bandFi_Cm_LPFCo1(:,:) = phase_bandFi_Cm_LPFCo(1,:,:);

%% plot - acc sensor filter
% figure;
% % set(gcf, 'position', [1100, 850, 1600, 600])
% subplot(221)
% semilogx(wout_op/2/pi, 20*log10(abs(mag_Co_band1)),'b','LineWidth', 2)
% grid on
% ylabel('magnitude (dB)')
% xlim([0 1000])
% ylim([-15 25])
% legend('Co band', 'Location','North')
% title('BPF for acc sensor')
% set(gca,'fontsize', 16);
% 
% subplot(223)
% semilogx(wout_Co_band/2/pi, phase_Co_band1,'b','LineWidth', 2)
% grid on
% ylabel('phase (deg)')
% xlabel('frequency (Hz)')
% xlim([0 1000])
% legend('Co band', 'Location','North')
% set(gca,'fontsize', 16);
% 
% subplot(222)
% semilogx(wout_Cm/2/pi, 20*log10(abs(mag_Cm1)),'k','LineWidth', 2)
% hold on
% semilogx(wout_Cm_bandCo/2/pi, 20*log10(abs(mag_Cm_bandCo1)),'b','LineWidth', 2)
% grid on
% ylabel('magnitude (dB)')
% xlim([0 1000])
% ylim([-15 25])
% legend('w/ Cm', 'w/ Cm & Co band', 'Location','North')
% title('system transfer function')
% set(gca,'fontsize', 16);
% 
% subplot(224)
% semilogx(wout_Cm/2/pi, phase_Cm1,'k','LineWidth', 2)
% hold on
% semilogx(wout_Cm_bandCo/2/pi, phase_Cm_bandCo1,'b','LineWidth', 2)
% grid on
% ylabel('phase (deg)')
% xlabel('frequency (Hz)')
% xlim([0 1000])
% legend('w/ Cm', 'w/ Cm & Co band', 'Location','South')
% set(gca,'fontsize', 16);

%% plot - current filter
% figure;
% set(gcf, 'position', [1100, 850, 800, 600])
% subplot(211)
% semilogx(wout_Fi_2nd/2/pi, 20*log10(abs(mag_Fi_2nd1)),'b','LineWidth', 2)
% hold on
% semilogx(wout_Fi_band/2/pi, 20*log10(abs(mag_Fi_band1)),'--r','LineWidth', 2)
% grid on
% ylabel('magnitude (dB)')
% xlim([0 1000])
% ylim([-15 5])
% legend('Fi 2nd', 'Fi band', 'Location','NorthWest')
% title('current sensor filter')
% set(gca,'fontsize', 16);
% 
% subplot(212)
% semilogx(wout_Fi_2nd/2/pi, phase_Fi_2nd1,'b','LineWidth', 2)
% hold on
% semilogx(wout_Fi_band/2/pi, phase_Fi_band1,'--r','LineWidth', 2)
% grid on
% ylabel('phase (deg)')
% xlabel('frequency (Hz)')
% xlim([0 1000])
% legend('Fi 2nd', 'Fi band', 'Location','NorthWest')
% set(gca,'fontsize', 16);
% 
% figure;
% set(gcf, 'position', [1100, 850, 800, 600])
% subplot(211)
% semilogx(wout_Cm_LPFCo/2/pi, 20*log10(abs(mag_Cm_LPFCo1)),'k','LineWidth', 2)
% hold on
% semilogx(wout_2ndFi_Cm_LPFCo/2/pi, 20*log10(abs(mag_2ndFi_Cm_LPFCo1)),'b','LineWidth', 2)
% semilogx(wout_bandFi_Cm_LPFCo/2/pi, 20*log10(abs(mag_bandFi_Cm_LPFCo1)),'--r','LineWidth', 2)
% grid on
% ylabel('magnitude (dB)')
% xlim([0 1000])
% ylim([-15 5])
% legend('w/ Cm & Co', 'w/ Cm & Co & Fi 2nd', 'w/ Cm & Co & Fi band', 'Location','South')
% title('system transfer function')
% set(gca,'fontsize', 16);
% 
% subplot(212)
% semilogx(wout_Cm_LPFCo/2/pi, phase_Cm_LPFCo1,'k','LineWidth', 2)
% hold on
% semilogx(wout_2ndFi_Cm_LPFCo/2/pi, phase_2ndFi_Cm_LPFCo1,'b','LineWidth', 2)
% semilogx(wout_bandFi_Cm_LPFCo/2/pi, phase_bandFi_Cm_LPFCo1,'--r','LineWidth', 2)
% grid on
% ylabel('phase (deg)')
% xlim([0 1000])
% % ylim([-90 90])
% legend('w/ Cm & Co', 'w/ Cm & Co & Fi 2nd', 'w/ Cm & Co & Fi band', 'Location','SouthWest')
% xlabel('frequency(Hz)')
% set(gca,'fontsize', 16);

%% plot - system transfer function
figure;
set(gcf, 'position', [1100, 850, 800, 600])
subplot(211)
semilogx(wout_op/2/pi, 20*log10(abs(mag_op1)),'k','LineWidth', 2)
hold on
semilogx(wout_Cm/2/pi, 20*log10(abs(mag_Cm1)),'b','LineWidth', 2)
semilogx(wout_Cm_LPFCo/2/pi, 20*log10(abs(mag_Cm_LPFCo1)),'c','LineWidth', 2)
semilogx(wout_2ndFi_Cm_LPFCo/2/pi, 20*log10(abs(mag_2ndFi_Cm_LPFCo1)),'r','LineWidth', 2)
semilogx(wout_bandFi_Cm_LPFCo/2/pi, 20*log10(abs(mag_bandFi_Cm_LPFCo1)),'--r','LineWidth', 2)
grid on
ylabel('magnitude (dB)')
xlim([0 1000])
ylim([-15 5])
legend('op', 'w/ Cm', 'w/ Cm & Co', 'w/ Cm & Co & Fi',  'w/ Cm & Co & Fi+LPF', 'Location','South')
title('system transfer function')
set(gca,'fontsize', 16);

subplot(212)
semilogx(wout_op/2/pi, 20*log10(abs(phase_op1)),'k','LineWidth', 2)
hold on
semilogx(wout_Cm/2/pi, phase_Cm1,'b','LineWidth', 2)
semilogx(wout_Cm_LPFCo/2/pi, phase_Cm_LPFCo1,'c','LineWidth', 2)
semilogx(wout_2ndFi_Cm_LPFCo/2/pi, phase_2ndFi_Cm_LPFCo1,'r','LineWidth', 2)
semilogx(wout_bandFi_Cm_LPFCo/2/pi, phase_bandFi_Cm_LPFCo1,'--r','LineWidth', 2)
grid on
ylabel('phase (deg)')
xlim([0 1000])
% ylim([-90 90])
% legend('op', 'w/ Cm', 'w/ Cm & Co', 'w/ Cm & Co & Fi',  'w/ Cm & Co & Fi+LPF', 'Location','SouthWest')
xlabel('frequency(Hz)')
set(gca,'fontsize', 16);

%%
% figure;
% set(gcf, 'position', [1100, 850, 800, 600])
% subplot(211)
% semilogx(wout_2ndFi_Cm_LPFCo/2/pi, 20*log10(abs(mag_2ndFi_Cm_LPFCo1)),'b','LineWidth', 2)
% grid on
% ylabel('magnitude (dB)')
% xlim([0 1000])
% % ylim([-15 5])
% title('controller & sensor filter & 2nd order current filter')
% set(gca,'fontsize', 16);
% 
% subplot(212)
% semilogx(wout_2ndFi_Cm_LPFCo/2/pi, phase_2ndFi_Cm_LPFCo1,'b','LineWidth', 2)
% grid on
% ylabel('phase (deg)')
% xlim([0 1000])
% % ylim([-90 90])
% xlabel('frequency(Hz)')
% set(gca,'fontsize', 16);