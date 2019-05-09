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

active = [0,pi/2,0];
%Defining workspace (necesarry given prismatic joint):
activeWorkVol = [-50*mm, 15000*mm, -50*mm, 50*mm, -50*mm, 150*mm];
homeWorkVol = activeWorkVol;
%% Defining DH parameters (m):
a_1 = 0; a_2 = 60*mm; a_3 = 40*mm; 
d_1 = 50*mm; d_2 = 0; d_3 = 0; 
alpha_1 = pi/2; alpha_2 = pi/2; alpha_3 = pi/2; 
theta_1 = 0; theta_2 = 0; theta_3 = 0; 
%% Defining Links:
L(1) = Link([theta_1 d_1 a_1 alpha_1 1]); % Just an offset from base.
L(2) = Link([theta_2 d_2 a_2 alpha_2 0]);
L(3) = Link([theta_3 d_3 a_2 alpha_3 0]);
% L(4) = Link([theta_2 d_2 a_2 alpha_2 0]);
% L(5) = Link([theta_3 d_3 a_2 alpha_3 0]);
% L(6) = Link([theta_2 d_2 a_2 alpha_2 0]);
% L(7) = Link([theta_3 d_3 a_2 alpha_3 0]);
% L(8) = Link([theta_2 d_2 a_2 alpha_2 0]);
% L(9) = Link([theta_3 d_3 a_2 alpha_3 0]);
%% Defining joint_limits:
q0 = [0,0];
q1 = [pi/4, 3*pi/4];
q2 = [-pi/4, pi/4];
% q3 = [-pi/4, pi/4];
% q4 = [-pi/4, pi/4];
% q5 = [-pi/4, pi/4];
% q6 = [-pi/4, pi/4];
% q7 = [-pi/4, pi/4];
% q8 = [-pi/4, pi/4];

my_q_lims = [q0; q1; q2]; %q3; q4; q5; q6; q7; q8];

home = zeros(1, length(my_q_lims));

%% Defining serial Link
RR_seg = SerialLink(L, 'name', 'RR segment');

%% Some conditional elements for testing...
just_a_plot = 0;
proximal_torque_and_heatmap_plot = 0;
wantScatterPlot = 1;


%% Plotting 
if(just_a_plot)
    figure(1);
    RR_seg.qlim = my_q_lims;
    RR_seg.teach;
    RR_seg.plot(active, 'workspace', activeWorkVol);
    hold on;
    title('RR modular segment')
    % 
    hold off;
end



%% Joint torques throughout ROM of each joint.
if(proximal_torque_and_heatmap_plot)
    
    % End-effector pose:
    EE_pose = RR_seg.fkine(home);

    q = zeros(1,length(my_q_lims));%joint variables
    g    = 9.81; %ms^-2
    m_EE = 10;   %kg (mass @ EE)

    F = [0;0;-g*m_EE;0;0;0]; %External forces/ moments experienced at EE

    EE_pose = RR_seg.fkine(home); %Pose of EE in home position;

    X = [];
    Y = [];
    Z = [];
    proximal_joint_tau = [];
    
    one_deg = pi/180;
    
    [joint_count, q_min_max] = size(my_q_lims);
    
%     for i = 1:joint_count
%         for theta_i = my_q_lims(i+1,1):one_deg:my_q_lims(i+1,2)
%             
%             
%         end
%     end
%     
    % Loop through all possible joint configurations (in steps of 1 deg).
    
    for th1 = pi/4:one_deg:3*pi/4
        for th2 = -pi/4:one_deg:pi/4
            for th3 = -pi/4:one_deg:pi/4
            %RR_seg.plot([0, th1, th2], 'workspace', activeWorkVol);

            % Update joint angles & compute resultant torques using jacobian.
            q(2) = th1;
            q(3) = th2; 
            q(4) = th3;
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
    end

    %% Plot EE translation with proximal joint torque encoded as color 
    figure, scatter3(X, Y, Z, 10, abs(proximal_joint_tau), 'filled');
    %axis([min(X), max(X), min(Y), max(Y), min(Z), max(Z)]);
    %axis('manual');
    c = colorbar;
    c.Label.String = '\bf Nm';
    title('\fontsize{10}Torque on proximal joint given 10kg at EE, for all possible joint configurations');
end

%% Author: Ian Howard; Edited by Alfred Wilmot for serpentine segment
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(wantScatterPlot)
    
    % Robot working area displayed by random uniformly distributed control parameters
    samples = 10000;
    
    theta0 = DrawFromUniformDistribution(0,0, samples);
    
    thetaRand = zeros(1,samples);
    q = zeros(samples, length(my_q_lims));
    for i = 1:length(my_q_lims)
        % draw samples from a uniform distribition between min and max limits
        % syntax: DrawFromUniformDistribution(min, max, samples)
%         theta1Rand = DrawFromUniformDistribution(q(2,1), q(2,2), samples);
%         theta2Rand = DrawFromUniformDistribution(q(3,1), q(3,2), samples);
        thetaRand = DrawFromUniformDistribution(my_q_lims(i,1), my_q_lims(i,2), samples);
        q(:,i) = thetaRand; 
    end
    % can use forward kinematics function in robotics toolbox too
    % calculate endpoint poisiton for random inputs
%     q = [theta0 theta1Rand theta2Rand];

    [TRand, All]= RR_seg.fkine(q);
   
    % extract homonegenous matrices from SE3 object
    TT=TRand.T;
    
    % extract position from homogeneous matrices
    TP = squeeze(TT( 1:3, 4, :));
    
    figure
    % plot the scattered points
    % plot3(x,y,z), where x, y and z are three vectors of the same length,
    % plots a line in 3-space through the points whose coordinates are the
    h = plot3(TP(1,:), TP(2,:), TP(3,:), 'r.');
    set(h, 'LineWidth', 3);
    RR_seg.plot(home,  'noshadow', 'workspace', activeWorkVol);
    
    robotName = "2-DOF Serpentine Segment";
    
    h=title(sprintf('%s:  Working area', robotName));
    set(h, 'FontSize', 20);
    
    % axis labels
    set(gca, 'FontSize', 20);
    
    % set the joint limits
    RR_seg.qlim = my_q_lims;
    
    % use interactive mode
    RR_seg.teach
    
    input('Hit return to proceed');
    close all
end



function values = DrawFromUniformDistribution(minVal, maxVal, samples)
% draw samples from a uniform distrubition between min and max limits

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 31/03/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

values =  (maxVal-minVal) * ( rand(samples, 1) - 0.5) + (minVal+maxVal)/2;

end

    