%% Haptic Human Robot Interface
close all; clear all; clc;

addpath('logs');

%% Lab1 ex 3.2
% a)+c) move hall sensor around
data_a = hri_load_logfile('lab1_hall_voltage_1.csv');
time_s = cumsum(data_a.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_a.encoder_paddle_pos__deg_;
hall_voltage = data_a.hall_voltage__V_;

% linear regression to find hall characteristics
p = polyfit(hall_voltage,encoder_pos,1);
slope = p(1);
intercept = p(2);

figure;
plot(hall_voltage, encoder_pos,...
     hall_voltage, slope*hall_voltage + intercept);
title('Lab1 ex 3.2.a) + c)');
legend('encoder pos','linear regression');
xlabel('hall voltage [V]');
ylabel('position [deg]');

% d)
maximum_linearity_error = max(abs(encoder_pos - (slope*hall_voltage + intercept)))

% b) noise when steady
data_b = hri_load_logfile('lab1_hall_pos_noise.csv');
time_s = cumsum(data_b.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_b.encoder_paddle_pos__deg_;
hall_voltage = data_b.hall_voltage__V_;

figure;
plot(time_s, hall_voltage);
title('Lab1 ex 3.2.b) - hall voltage std');
xlabel('time [s]');
ylabel('hall voltage [V]');

hall_voltage_std = std(hall_voltage)
hall_pos_std = std(slope*hall_voltage + intercept)

%% Lab1 ex 3.3
% a) + b) speed noise
lab1_hall_speed_noise = hri_load_logfile('lab1_hall_speed_noise.csv');
time_s = cumsum(lab1_hall_speed_noise.timestep__us_*10^(-6)); % us -> s
encoder_speed = lab1_hall_speed_noise.encoder_paddle_speed__deg_s_;
hall_speed_over_1_p = lab1_hall_speed_noise.hall_paddle_speed_over_1_p__deg_s_;
hall_speed_over_2_p = lab1_hall_speed_noise.hall_paddle_speed_over_2_p__deg_s_;


figure;
plot(time_s, hall_speed_over_1_p,...
     time_s, hall_speed_over_2_p);
title('Lab1 ex 3.3.a) - hall speed std');
legend('differentiated over 1 period','differentiated over 2 periods');
xlabel('time [s]');
ylabel('angular speed [deg/s]');

hall_speed_over_1_p_std = std(hall_speed_over_1_p)
hall_speed_over_2_p_std = std(hall_speed_over_2_p)
% if we differentiate the speed over 2 periods instead of 1
% the noise is almost divided by 2

% c) + d) acceleration noise
lab1_hall_accel_noise = hri_load_logfile('lab1_hall_accel_noise.csv');
time_s = cumsum(lab1_hall_accel_noise.timestep__us_*10^(-6)); % us -> s
hall_accel_over_1_p = lab1_hall_accel_noise.hall_paddle_accel_over_1_p__deg_s_2_;
hall_accel_over_2_p = lab1_hall_accel_noise.hall_paddle_accel_over_2_p__deg_s_2_;


figure;
plot(time_s, hall_accel_over_1_p,...
     time_s, hall_accel_over_2_p);
title('Lab1 ex 3.3.a) - hall acceleration std');
legend('differentiated over 1 period','differentiated over 2 periods');
xlabel('time [s]');
ylabel('angular acceleration [deg/s^2]');

hall_acceleration_over_1_p_std = std(hall_accel_over_1_p)
hall_acceleration_over_2_p_std = std(hall_accel_over_2_p)
% if we differentiate the speed over 2 periods instead of 1
% the noise is almost divided by 2









