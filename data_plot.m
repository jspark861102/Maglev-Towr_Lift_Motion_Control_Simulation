%% accleration plot ====================================================
figure;
set(gcf, 'position', [20, 50, 1000, 1200])
subplot(3,2,1)
plot(t,ddroll_cm_m,'b','LineWidth',3)
hold on
plot(t,ddroll_cm,'--r','LineWidth',3)
plot(t,ddroll_ref,':k','LineWidth',3)
title('ddroll')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('rad/sec^2')
set(gca,'fontsize', 16);
% ylim([-5 5])

subplot(3,2,3)
plot(t,ddpitch_cm_m,'b','LineWidth',3)
hold on
plot(t,ddpitch_cm,'--r','LineWidth',3)
plot(t,ddpitch_ref,':k','LineWidth',3)
title('ddpitch')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('rad/sec^2')
set(gca,'fontsize', 16);
% ylim([-5 5])

subplot(3,2,5)
plot(t,ddyaw_cm_m,'b','LineWidth',3)
hold on
plot(t,ddyaw_cm,'--r','LineWidth',3)
plot(t,ddyaw_ref,':k','LineWidth',3)
title('ddyaw')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('rad/sec^2')
set(gca,'fontsize', 16);
% ylim([-5 5])

subplot(3,2,2)
plot(t,ddx_cm_m,'b','LineWidth',3)
hold on
plot(t,ddx_cm,'--r','LineWidth',3)
plot(t,ddx_ref,':k','LineWidth',3)
title('ddx')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('m/s^2')
set(gca,'fontsize', 16);
% ylim([-5 5])

subplot(3,2,4)
plot(t,ddy_cm_m,'b','LineWidth',3)
hold on
plot(t,ddy_cm,'--r','LineWidth',3)
plot(t,ddy_ref,':k','LineWidth',3)
title('ddy')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('m/s^2')
set(gca,'fontsize', 16);
% ylim([-5 5])


%% velocity ============================================
figure;
set(gcf, 'position', [1100, 50, 1000, 1200])
subplot(3,2,1)
plot(t,droll_cm_m,'b','LineWidth',3)
hold on
plot(t,droll_cm,'--r','LineWidth',3)
plot(t,droll_ref,':k','LineWidth',3)
title('droll')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('rad/sec')
set(gca,'fontsize', 16);
% ylim([-0.5 0.5])

subplot(3,2,3)
plot(t,dpitch_cm_m,'b','LineWidth',3)
hold on
plot(t,dpitch_cm,'--r','LineWidth',3)
plot(t,dpitch_ref,':k','LineWidth',3)
title('dpitch')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('rad/sec')
set(gca,'fontsize', 16);
% ylim([-0.5 0.5])

subplot(3,2,5)
plot(t,dyaw_cm_m,'b','LineWidth',3)
hold on
plot(t,dyaw_cm,'--r','LineWidth',3)
plot(t,dyaw_ref,':k','LineWidth',3)
title('dyaw')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('rad/sec')
set(gca,'fontsize', 16);
% ylim([-0.5 0.5])

subplot(3,2,2)
plot(t,dx_cm_m,'b','LineWidth',3)
hold on
plot(t,dx_cm,'--r','LineWidth',3)
plot(t,dx_ref,':k','LineWidth',3)
title('dx')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('m/s')
set(gca,'fontsize', 16);
% ylim([-0.5 0.5])

subplot(3,2,4)
plot(t,dy_cm_m,'b','LineWidth',3)
hold on
plot(t,dy_cm,'--r','LineWidth',3)
plot(t,dy_ref,':k','LineWidth',3)
title('dy')
legend('estimated cm-m','real cm', 'ref cm');
xlabel('time(sec)')
ylabel('m/s')
set(gca,'fontsize', 16);
% ylim([-0.5 0.5])
