%% Haptic Human Robot Interface
close all; clear all; clc;

addpath('logs');

%% Lab Specialization Speed Plots
data_a = hri_load_logfile('labSpec_speeds.csv');
time_s = cumsum(data_a.timestep__us_*10^(-6)); % us -> s

euler = data_a.encoder_paddle_speed_over_1_p__deg_s_;
euler_filtered = data_a.encoder_paddle_speed_filt__deg_s_;
foaw = data_a.hapt_foaw_speed__deg_s_;
foaw_filtered = data_a.hapt_foaw_filtered_speed__deg_s_;
levant = data_a.levant_speed__deg_s_;
levant_filtered = data_a.hapt_levant_filtered_speed__deg_s_;
kalman = data_a.kalman_speed__deg_s_;

figure;
plot(time_s, euler,'y',...
     time_s, euler_filtered,'m',...
     time_s, foaw_filtered,'b',...
     time_s, levant_filtered,'g',...
     time_s, kalman,'r');
title('Lab Specialization - Velocity Estimates');
legend('Euler',...
       'Euler filtered',...
       'FOAW filtered',...
       'Levant filtered',...
       'Kalman');
xlabel('Time [s]');
ylabel('Speed [deg/s]');





