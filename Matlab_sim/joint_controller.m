clc;
close all;
clear all;


theta_0 = 0;
theta_1 = [-pi/4:0.01:pi/4];
k = 1;
R_0 = (2^0.5)* k * (1 - cos((pi/4) + theta_0));
R_1 = (2^0.5)* k * (1 - cos((pi/4) + theta_1));


l_0 = 2 * k * sin(( (pi/4) + theta_0)/2);
l_1 = 2 * k * sin(( (pi/4) + theta_1)/2);

delta_l = l_0 - l_1;

delta_R = R_0 - R_1;


plot(theta_1, delta_l);
hold on;

plot(theta_1, delta_R);

legend("deta_l","delta_R");
