%---------%
%%PROJ324%%
%---------%
%%
% DH parameters of single segment for basic simulation.
%Preamble:
startup_rvc;
close all; clear all; clc;
mm = 10^(-3);
pi = 3.14159;
%%
%----------------%
%DH convention...
%----------------%
%NOTE: all measurements are in meters (as this significantly reduces robot-plot
%rendering over-head compared to using cm).
%Defining joint configurations.
home = [0,0,0];
active = [0,0,0];
%Defining workspace (necesarry given prismatic joint):
homeWorkVol = [0, 200*mm, -100*mm, 100*mm, -100*mm, 100*mm];
activeWorkVol = [-100*mm, 100*mm, -100*mm, 100*mm, -100*mm, 200*mm];
%% Defining DH parameters (m):
a_1 = 0; a_2 = 60*mm; a_3 = 40*mm; 
d_1 = 50*mm; d_2 = 0; d_3 = 0; 
alpha_1 = pi/2; alpha_2 = pi/2; alpha_3 = 0;%pi/2; 
theta_1 = pi; theta_2 = -pi/2; theta_3 = -pi/2; 
%% Defining Links:
L(1) = Link([theta_1 d_1 a_1 alpha_1 0]);
L(2) = Link([theta_2 d_2 a_2 alpha_2 0]);
L(3) = Link([theta_3 d_3 a_3 alpha_3 0]);

figure(1);
RR_Segment = SerialLink(L, 'name', 'RR segment');
RR_Segment.plot(active, 'workspace', activeWorkVol);
hold on;
title('RR modular segment')

hold off;
%% End-effector pose:
EE_pose = RR_Segment.fkine(home);