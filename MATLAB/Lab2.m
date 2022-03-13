%% Haptic Human Robot Interface
close all; clear all; clc;

addpath('logs');

%% Lab2

% increase motor torque until paddle moves
% data_f = hri_load_logfile('lab2_friction+.csv');
% time_s = cumsum(data_f.timestep__us_*10^(-6)); % us -> s
% friction_moment = data_f.motor_torque__N_m_;
% max_friction_moment = max(friction_moment)
% 
% figure;
% plot(time_s, friction_moment);
% title('Lab2 - friction values in the positive direction');
% xlabel('time [s]');
% ylabel('dry friction moment [Nm]');

% decrease motor torque until paddle moves
% data_f = hri_load_logfile('lab2_friction-.csv');
% time_s = cumsum(data_f.timestep__us_*10^(-6)); % us -> s
% friction_moment = data_f.motor_torque__N_m_;
% min_friction_moment = min(friction_moment)
% 
% figure;
% plot(time_s, friction_moment);
% title('Lab2 - friction values in the negative direction');
% xlabel('time [s]');
% ylabel('dry friction moment [Nm]');





% step response with position controller on encoder angle
data_a = hri_load_logfile('lab2_encoder_step_response.csv');
time_s = cumsum(data_a.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_a.encoder_paddle_pos__deg_;
encoder_target_pos = data_a.Target_Angle__deg_;

figure;
plot(time_s, encoder_target_pos, time_s, encoder_pos);
title('Lab2 - step response with position controller on encoder angle');
legend('target angle','encoder angle');
xlabel('time [s]');
ylabel('position [deg]');

% step response with position controller on hall angle
data_b = hri_load_logfile('lab2_hall_step_response.csv');
time_s = cumsum(data_b.timestep__us_*10^(-6)); % us -> s
hall_pos = data_b.hall_paddle_pos__deg_;
hall_target_pos = data_b.Target_Angle__deg_;

figure;
plot(time_s, hall_target_pos, time_s, hall_pos);
title('Lab2 - step response with position controller on hall angle');
legend('target angle','hall angle');
xlabel('time [s]');
ylabel('position [deg]');







% HALL ZN TUNING
data_c = hri_load_logfile('lab2_hall_ZN_tuning.csv');
time_s = cumsum(data_c.timestep__us_*10^(-6)); % us -> s
hall_pos = data_c.hall_paddle_pos__deg_;
hall_target_pos = data_c.Target_Angle__deg_;

% figure;
% plot(time_s, hall_target_pos, time_s, hall_pos);
% title('Lab2 - determine KU and Tu from step response using hall angle');
% legend('target angle','hall angle');
% xlabel('time [s]');
% ylabel('position [deg]');



% HALL ZN TUNED
data_d = hri_load_logfile('lab2_hall_ZN_tuned.csv');
time_s = cumsum(data_d.timestep__us_*10^(-6)); % us -> s
hall_pos = data_d.hapt_hallPaddleAngleFilt__deg_;
hall_target_pos = data_d.Target_Angle__deg_;

figure;
plot(time_s, hall_target_pos, time_s, hall_pos);
title('Lab2 - step response using ZN tuned controller on hall angle');
legend('target angle','hall angle');
xlabel('time [s]');
ylabel('position [deg]');


% ENCODER ZN TUNING
data_e = hri_load_logfile('lab2_encoder_ZN_tuning.csv');
time_s = cumsum(data_e.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_e.encoder_paddle_pos__deg_;
encoder_target_pos = data_e.Target_Angle__deg_;

% figure;
% plot(time_s, encoder_target_pos, time_s, encoder_pos);
% title('Lab2 - determine KU and Tu from step response using encoder angle');
% legend('target angle','encoder angle');
% xlabel('time [s]');
% ylabel('position [deg]');



% ENCODER ZN TUNED
data_f = hri_load_logfile('lab2_encoder_ZN_tuned.csv');
time_s = cumsum(data_f.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_f.encoder_paddle_pos__deg_;
encoder_target_pos = data_f.Target_Angle__deg_;

figure;
plot(time_s, encoder_target_pos, time_s, encoder_pos);
title('Lab2 - step response using ZN tuned controller on encoder angle');
legend('target angle','encoder angle');
xlabel('time [s]');
ylabel('position [deg]');



% Ziegler-Nichols gains

% ZN hall real paddle
% Ku = 0.01345;
% Tu = 0.045;
% ZN encoder real paddle
Ku = 0.07;
Tu = 0.023;
Kp = 0.6*Ku
Ki = Kp/(0.5*Tu)
Kd = Kp*0.125*Tu

