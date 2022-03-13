%% Haptic Human Robot Interface
close all; clear all; clc;

addpath('logs');

%% Lab3

% increase motor torque until paddle moves
data_f = hri_load_logfile('lab3_static_dynamic_friction_pos.csv');
time_s = cumsum(data_f.timestep__us_*10^(-6)); % us -> s
friction_moment = data_f.motor_torque__N_m_;

figure;
plot(time_s, friction_moment);
title('Lab3 - friction values in the positive direction');
xlabel('Time [s]');
ylabel('Dry Friction Moment [Nm]');

% decrease motor torque until paddle moves
data_f = hri_load_logfile('lab3_static_dynamic_friction_neg.csv');
time_s = cumsum(data_f.timestep__us_*10^(-6)); % us -> s
friction_moment = data_f.motor_torque__N_m_;

figure;
plot(time_s, friction_moment);
title('Lab3 - friction values in the negative direction');
xlabel('Time [s]');
ylabel('Dry Friction Moment [Nm]');


% identify the noise in the filtered encoder speed
data_f = hri_load_logfile('lab3_encoder_speed_filt_noise.csv');
time_s = cumsum(data_f.timestep__us_*10^(-6)); % us -> s
encoder_speed_filt = data_f.encoder_paddle_speed_filt__deg_s_;

figure;
plot(time_s, encoder_speed_filt);
title('Lab3 - filtered encoder speed');
xlabel('Time [s]');
ylabel('Speed [deg/s]');


%% DRY FRICTION COMPENSATION
data_f = hri_load_logfile('lab3_compensated_dry.csv');
time_s = cumsum(data_f.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_f.encoder_paddle_pos__deg_;

figure;
plot(time_s, encoder_pos);
title('Lab3 - Dry Friction Compensated');
xlabel('Time [s]');
ylabel('Angle [deg]');

%% VISCOUS FRICTION COMPENSATION
data_f = hri_load_logfile('lab3_compensated_viscous.csv');
time_s = cumsum(data_f.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_f.encoder_paddle_pos__deg_;

figure;
plot(time_s, encoder_pos);
title('Lab3 - Dry and Viscous Friction Compensated');
xlabel('Time [s]');
ylabel('Angle [deg]');

%% GRAVITY COMPENSATION
data_f = hri_load_logfile('lab3_compensated_gravity.csv');
time_s = cumsum(data_f.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_f.encoder_paddle_pos__deg_;

figure;
plot(time_s, encoder_pos);
title('Lab3 - Dry and Viscous Friction and Gravity Compensated');
xlabel('Time [s]');
ylabel('Angle [deg]');

