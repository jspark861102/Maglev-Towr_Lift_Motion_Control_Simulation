%% Algebraic solution
function [ddx_cm, ddy_cm, ddroll_cm, ddpitch_cm, ddyaw_cm] = center_motion_estimation_from_acc_sensor(ha, wa, acc1x, acc2x, acc3x, acc4x, acc1y, acc2y, acc3y, acc4y)


ddx_cm = (acc1x + acc2x + acc3x + acc4x) / 4;
ddy_cm = (acc1y + acc2y + acc3y + acc4y) / 4;

ddroll_cm  = (-acc1y + acc2y - acc3y + acc4y) / (4*ha);
ddpitch_cm = ( acc1x - acc2x + acc3x - acc4x) / (4*ha);
ddyaw_cm   = ( acc1x + acc2x - acc3x - acc4x) / (4*wa);

end