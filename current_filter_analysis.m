%  save('current_filter_analysis.mat','t','ref','nofiltered','filtered')
load current_filter_analysis.mat

%current filter cut off frequency : 10Hz

figure;
set(gcf, 'position', [20, 50, 1000, 1200])
subplot(3,2,1)
plot(t,ref(:,1),'--k','LineWidth',3)
hold on
plot(t,nofiltered(:,1),'r','LineWidth',2)
plot(t,filtered(:,1),'b','LineWidth',2)
grid on
xlabel('time(sec)')
ylabel('deg')
title('ddroll')
set(gca,'fontsize', 16);
legend('ref','dX w/ nofilted U', 'dX w/ filted U')

subplot(3,2,3)
plot(t,ref(:,2),'--k','LineWidth',3)
hold on
plot(t,nofiltered(:,2),'r','LineWidth',2)
plot(t,filtered(:,2),'b','LineWidth',2)
grid on
xlabel('time(sec)')
ylabel('deg')
title('ddpitch')
set(gca,'fontsize', 16);
legend('ref','dX w/ nofilted U', 'dX w/ filted U')

subplot(3,2,5)
plot(t,ref(:,3),'--k','LineWidth',3)
hold on
plot(t,nofiltered(:,3),'r','LineWidth',2)
plot(t,filtered(:,3),'b','LineWidth',2)
grid on
xlabel('time(sec)')
ylabel('deg')
title('ddyaw')
set(gca,'fontsize', 16);
legend('ref','dX w/ nofilted U', 'dX w/ filted U')

subplot(3,2,2)
plot(t,ref(:,4),'--k','LineWidth',3)
hold on
plot(t,nofiltered(:,4),'r','LineWidth',2)
plot(t,filtered(:,4),'b','LineWidth',2)
grid on
xlabel('time(sec)')
ylabel('m')
title('ddx')
set(gca,'fontsize', 16);
legend('ref','dX w/ nofilted U', 'dX w/ filted U')

subplot(3,2,4)
plot(t,ref(:,5),'--k','LineWidth',3)
hold on
plot(t,nofiltered(:,5),'r','LineWidth',2)
plot(t,filtered(:,5),'b','LineWidth',2)
grid on
xlabel('time(sec)')
ylabel('m')
title('ddy')
set(gca,'fontsize', 16);
legend('ref','dX w/ nofilted U', 'dX w/ filted U')