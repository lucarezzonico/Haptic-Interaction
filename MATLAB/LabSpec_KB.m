%% Haptic Human Robot Interface
close all; clear all; clc;

addpath('logs');

%% Lab Specialization K-B PLOTS

B_50hz = [0.000,0.002,0.004,0.006,0.008,0.010,0.012,0.014,0.016];
K_50hz = [1.8,2.4,2.8,3.0,2.4,2.2,1.8,1.2,0.1];
B_50hz_foaw = [0.000,0.002,0.004,0.006,0.008,0.010,0.012,0.014,0.016,0.018];
K_50hz_foaw = [5.4,5.8,5.8,5.4,4.8,4.4,2.4,1.6,0.8,0.2];
B_50hz_levant = [0.000,0.002,0.004,0.006,0.008,0.010,0.012,0.014,0.016,0.018,0.020,0.022,0.024];
K_50hz_levant = [5.2,5.8,6.2,6.2,5.8,5.6,5.2,4.8,4.0,3.2,2.4,0.8,0.2];
B_50hz_kalman = [0.000,0.002,0.004,0.006,0.008,0.010,0.012,0.014,0.016,0.018,0.020];
K_50hz_kalman = [3.0,4.0,4.0,3.8,3.2,2.4,2.0,1.2,0.8,0.4,0.1]; % the limit is way smoother, delimitation not as clear as for the others


figure;
plot(B_50hz, K_50hz, 'r-x',...
     B_50hz_foaw, K_50hz_foaw, 'g-x',...
     B_50hz_levant, K_50hz_levant, 'b-x',...
     B_50hz_kalman, K_50hz_kalman, 'm-x');
title('Lab Specialization -  K-B Plot of the Haptic Paddle');
legend('Euler','FOAW','Levant','Kalman');
xlabel('Virtual Damping B [Ns/mm]');
ylabel('Virtual Stiffness K [N/mm]');


