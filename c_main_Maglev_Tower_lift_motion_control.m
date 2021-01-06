clear all
close all
clc

%% MTL parameters ========================================== 
a_parameters_Maglev_Tower_lift_motion

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% motion control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% discrete control gain ==========================================
if isdiscrete == 1
    ki_roll =  ki_roll*dt;
    ki_pitch = ki_pitch*dt;
    ki_yaw =   ki_yaw*dt;
    ki_x =     ki_x*dt;
    ki_y =     ki_y*dt;
    
    kp_roll =  kp_roll*dt;
    kp_pitch = kp_pitch*dt;
    kp_yaw =   kp_yaw*dt;
    kp_x =     kp_x*dt;
    kp_y =     kp_y*dt;
end

%% reference ==========================================
droll_ref = droll_ref_mag*sin(droll_ref_frequency*t);    
dpitch_ref = dpitch_ref_mag*sin(dpitch_ref_frequency*t);    
dyaw_ref = dyaw_ref_mag*sin(dyaw_ref_frequency*t);    
dx_ref = dx_ref_mag*sin(dx_ref_frequency*t);    
dy_ref = dy_ref_mag*sin(dy_ref_frequency*t); 

ddroll_ref =  droll_ref_mag *  droll_ref_frequency *  cos(droll_ref_frequency *  t);    
ddpitch_ref = dpitch_ref_mag * dpitch_ref_frequency * cos(dpitch_ref_frequency * t);    
ddyaw_ref =   dyaw_ref_mag *   dyaw_ref_frequency *   cos(dyaw_ref_frequency *   t);    
ddx_ref =     dx_ref_mag *     dx_ref_frequency *     cos(dx_ref_frequency *     t);    
ddy_ref =     dy_ref_mag *     dy_ref_frequency *     cos(dy_ref_frequency *     t);  

%% Simulation ====================================================
for i = 1 : Ns        
    %cg motion from acc sensor (da is assumed as 0 for motion estimation, because angle is needed for coordinate transformation)
    [ddx_cm_m(i), ddy_cm_m(i), ddroll_cm_m(i), ddpitch_cm_m(i), ddyaw_cm_m(i)] = center_motion_estimation_from_acc_sensor(ha, wa, acc1x(i), acc2x(i), acc3x(i), acc4x(i), acc1y(i), acc2y(i), acc3y(i), acc4y(i));

    %obtain velocities at cm to use I control gain    
    if i > 1        
        droll_cm_m(i)  = droll_cm_m(i-1)  + ddroll_cm_m(i)*dt;
        dpitch_cm_m(i) = dpitch_cm_m(i-1) + ddpitch_cm_m(i)*dt;
        dyaw_cm_m(i)   = dyaw_cm_m(i-1)   + ddyaw_cm_m(i)*dt;
        dx_cm_m(i)     = dx_cm_m(i-1)     + ddx_cm_m(i)*dt;
        dy_cm_m(i)     = dy_cm_m(i-1)     + ddy_cm_m(i)*dt;
    end             

    %controller    
    v_ddroll(i)  = -kp_roll  * (ddroll_cm_m(i)  - ddroll_ref(i))    -ki_roll  * (droll_cm_m(i)  - droll_ref(i));
    v_ddpitch(i) = -kp_pitch * (ddpitch_cm_m(i) - ddpitch_ref(i))   -ki_pitch * (dpitch_cm_m(i) - dpitch_ref(i));
    v_ddyaw(i)   = -kp_yaw   * (ddyaw_cm_m(i)   - ddyaw_ref(i))     -ki_yaw   * (dyaw_cm_m(i)   - dyaw_ref(i));    
    v_ddx(i)     = -kp_x     * (ddx_cm_m(i)     - ddx_ref(i))       -ki_x     * (dx_cm_m(i)     - dx_ref(i));
    v_ddy(i)     = -kp_y     * (ddy_cm_m(i)     - ddy_ref(i))       -ki_y     * (dy_cm_m(i)     - dy_ref(i));

    %system
    if isdiscrete == 1        
        %control input allocation to 8 magnetics
        u(:,i) = pinv(Bdu) * [v_ddroll(i); v_ddpitch(i); v_ddyaw(i); v_ddx(i); v_ddy(i)];         

        %plant (real carrier motion)    
        X(:,i+1)    = Ad*X(:,i) + Bd * u(:,i);   

    else    
        %control input allocation to 8 magnetics
        u(:,i) = pinv(Bu) * [v_ddroll(i); v_ddpitch(i); v_ddyaw(i); v_ddx(i); v_ddy(i)];           

        %plant (real carrier motion)    
        dX(:,i+1)   = A*X(:,i) + B * u(:,i); 
        X(:,i+1)    = X(:,i) + dX(:,i+1) * dt;
    end

    %make next step acceleration sensor signal from plant state 
    if i < Ns
        droll_cm(i+1)  = X(1,i+1);
        dpitch_cm(i+1) = X(2,i+1);
        dyaw_cm(i+1)   = X(3,i+1);
        dx_cm(i+1)     = X(4,i+1);    
        dy_cm(i+1)     = X(5,i+1);

        if isdiscrete == 1        
            ddroll_cm(i+1)  = (X(1,i+1) - X(1,i))/dt;
            ddpitch_cm(i+1) = (X(2,i+1) - X(2,i))/dt;
            ddyaw_cm(i+1)   = (X(3,i+1) - X(3,i))/dt;
            ddx_cm(i+1)     = (X(4,i+1) - X(4,i))/dt;    
            ddy_cm(i+1)     = (X(5,i+1) - X(5,i))/dt;    
        else        
            ddroll_cm(i+1)  = dX(1,i+1);
            ddpitch_cm(i+1) = dX(2,i+1);
            ddyaw_cm(i+1)   = dX(3,i+1);
            ddx_cm(i+1)     = dX(4,i+1);    
            ddy_cm(i+1)     = dX(5,i+1);    
        end

        acc1x(i+1) = ddx_cm(i+1) + ha*ddpitch_cm(i+1) + wa*ddyaw_cm(i+1) +  0.5*(rand(1)-0.5)*isnoise;
        acc2x(i+1) = ddx_cm(i+1) - ha*ddpitch_cm(i+1) + wa*ddyaw_cm(i+1) +  0.5*(rand(1)-0.5)*isnoise;
        acc3x(i+1) = ddx_cm(i+1) + ha*ddpitch_cm(i+1) - wa*ddyaw_cm(i+1) +  0.5*(rand(1)-0.5)*isnoise;
        acc4x(i+1) = ddx_cm(i+1) - ha*ddpitch_cm(i+1) - wa*ddyaw_cm(i+1) +  0.5*(rand(1)-0.5)*isnoise;
        acc1y(i+1) = ddy_cm(i+1) - ha*ddroll_cm(i+1) +  0.5*(rand(1)-0.5)*isnoise;
        acc2y(i+1) = ddy_cm(i+1) + ha*ddroll_cm(i+1) +  0.5*(rand(1)-0.5)*isnoise;
        acc3y(i+1) = ddy_cm(i+1) - ha*ddroll_cm(i+1) +  0.5*(rand(1)-0.5)*isnoise;
        acc4y(i+1) = ddy_cm(i+1) + ha*ddroll_cm(i+1) +  0.5*(rand(1)-0.5)*isnoise;  
    end
end

%% data plot ============================================
data_plot