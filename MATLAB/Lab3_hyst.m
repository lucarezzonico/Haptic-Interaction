%% Haptic Human Robot Interface
close all; clear all; clc;

addpath('logs');

%% Lab3 K-B PLOTS

speed = [-10,-3,-3,3,3,10];
torque = [-0.00122,-0.00122,0.00148,0.00148,0.00148,0.00148];
torque2 = [-0.00122,-0.00122,-0.00122,-0.00122,0.00148,0.00148];

figure;
plot(speed, torque, 'b-', speed, torque2, 'b-');
title('Lab3 - Torque-Speed Hysteresis');
xlabel('Speed [deg/s]');
ylabel('Torque [Nm]');


