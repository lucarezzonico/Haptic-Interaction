%% Haptic Human Robot Interface
close all; clear all; clc;

addpath('logs');

%% Lab0 ex 2.4.1.3) plot paddle position with spring
% array
% data_a = table2array(readtable('lab0_encoder_paddle_pos_deg_spring.csv', 'HeaderLines',1));  % skips the first three rows of data
% time_s = cumsum(data_a(2:end,4))*10^(-6); % us -> s
% pos_deg = data_a(2:end,6);

% structure
data_a = hri_load_logfile('lab0_encoder_paddle_pos_deg_spring.csv');
time_s = cumsum(data_a.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_a.encoder_paddle_pos__deg_;

figure;
plot(time_s, encoder_pos);
title('Lab0 ex 2.4.1.3) - Paddle position with spring');
xlabel('time [s]');
ylabel('position [deg]');

%% Lab0 ex 2.4.2.1) plot paddle angular speed

% numeric differentiation
encoder_speed_deg_s = data_a.encoder_paddle_speed__deg_s_;

figure;
plot(time_s, encoder_speed_deg_s, 'r', time_s, encoder_pos,'b');

%% Lab0 ex 2.4.2.3) plot paddle position with damping
% array
% data_b = table2array(readtable('lab0_encoder_paddle_pos_deg_damping.csv', 'HeaderLines',1));  % skips the first three rows of data
% time_s = cumsum(data_b(2:end,4))*10^(-6); % us -> s
% pos_deg = data_b(2:end,6);

% structure
data_b = hri_load_logfile('lab0_encoder_paddle_pos_deg_damping.csv');
time_s = cumsum(data_b.timestep__us_*10^(-6)); % us -> s
encoder_pos = data_b.encoder_paddle_pos__deg_;

figure;
plot(time_s, encoder_pos);
title('Lab0 ex 2.4.1.3) - Paddle position with damping');
xlabel('time [s]');
ylabel('position [deg]');

