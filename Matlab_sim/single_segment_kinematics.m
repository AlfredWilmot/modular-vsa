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
home = [0,pi/2,0];
active = [0,pi/2,0];
%Defining workspace (necesarry given prismatic joint):
homeWorkVol = [0, 100*mm, -100*mm, 100*mm, -100*mm, 100*mm];
activeWorkVol = [-100*mm, 100*mm, -100*mm, 100*mm, -100*mm, 100*mm];
%% Defining DH parameters (m):
a_1 = 0; a_2 = 60*mm; a_3 = 40*mm; 
d_1 = 50*mm; d_2 = 0; d_3 = 0; 
alpha_1 = pi/2; alpha_2 = pi/2; alpha_3 = pi/2; 
theta_1 = 0; theta_2 = 0; theta_3 = 0; 
%% Defining Links:
L(1) = Link([theta_1 d_1 a_1 alpha_1 1]); % Just an offset from base.
L(2) = Link([theta_2 d_2 a_2 alpha_2 0]);
L(3) = Link([theta_3 d_3 a_3 alpha_3 0]);

%% Plotting 
% figure(1);
RR_seg = SerialLink(L, 'name', 'RR segment');
% RR_seg.plot(active, 'workspace', activeWorkVol);
% hold on;
% title('RR modular segment')
% 
% hold off;

%% End-effector pose:
EE_pose = RR_seg.fkine(home);

%% Joint torques throughout ROM of each joint.

q = [0,0,0]; %joint variables
g    = 9.81; %ms^-2
m_EE = 10;   %kg (mass @ EE)

F = [0;0;-g*m_EE;0;0;0]; %External forces/ moments experienced at EE

EE_pose = RR_seg.fkine(home); %Pose of EE in home position;

X = [];
Y = [];
Z = [];
proximal_joint_tau = [];

% Loop through all possible joint configurations (in steps of 1 deg).
one_deg = pi/180;
for th1 = pi/4:one_deg:3*pi/4
    for th2 = -pi/4:one_deg:pi/4
        %RR_seg.plot([0, th1, th2], 'workspace', activeWorkVol);
        
        % Update joint angles & compute resultant torques using jacobian.
        q(2) = th1;
        q(3) = th2; 
        J = jacob0(RR_seg, q);
        joint_torques = (-J'*F)';

        
        % Store EE position for each joint configuration, and torque at
        % proximal joint 
        EE_pose = RR_seg.fkine(q);
        X = [X, EE_pose.t(1)];
        Y = [Y, EE_pose.t(2)];
        Z = [Z, EE_pose.t(3)];
        proximal_joint_tau = [proximal_joint_tau, joint_torques(2)];
    end
end

%% Plot EE translation with proximal joint torque encoded as color 
figure, scatter3(X, Y, Z, 10, abs(proximal_joint_tau), 'filled');
%axis([min(X), max(X), min(Y), max(Y), min(Z), max(Z)]);
%axis('manual');
c = colorbar;
c.Label.String = '\bf Nm';
title('\fontsize{10}Torque on proximal joint given 10kg at EE, for all possible joint configurations');



    