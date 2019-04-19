clc;
close all;
clear all;

% Assumptions for simple controller:
% -> non-antagonist setup (only considering one motor driving joint)
% -> cable is treated as a non-elastic linkage (hence single motor can
%       drive joint in both directions).
% -> motor-shaft diameter is constant (this is in fact a function of joint
%       position as currently the cable will spool over itself).

%% Comparing the system response between two derivations, starting from resting position (theta_0)
% moving towards all possible joint positoins (theta_1) ranging from -pi/4
% to pi/4.
theta_0 = 0;
theta_1 = [-pi/4:0.01:pi/4];

% arbitrary constant (lever-length for cable driving joint).
k = 1;

% Law of cosines derivation.
R_0 = (2^0.5)* k * (1 - cos((pi/4) + theta_0)).^0.5;
R_1 = (2^0.5)* k * (1 - cos((pi/4) + theta_1)).^0.5;
delta_R = R_0 - R_1;

% Extended derivation to further simplify equation (using half-angle identity)
l_0 = 2 * k * sin(( (pi/4) + theta_0)/2);
l_1 = 2 * k * sin(( (pi/4) + theta_1)/2);
delta_l = l_0 - l_1;


% the deltas are the difference in linkage-lengths (i.e. cables) due to moving from one
% joint position to another.

plot(theta_1, delta_l);
hold on;
plot(theta_1, delta_R);

legend("deta_l","delta_R");
title("Approximately linear-response of cable-length variations");
xlabel("desired joint angle offset from 0 degrees (deg)");
ylabel("change in cable-length function output (mm)");





%% What motor velocity model is needed in order to efficiently drive joint towards desired position.

clc;
close all;
clear all;


% make recursive function that compares "current" joint angle against
% "target" joint angle?

% dummy set of "current" joint values as they go from 0 to theta_1.
theta_0 = [-pi/4:0.01:pi/4];

% simulate time steps.
delta_t = 0.1;

% the "desired" joint angle.
theta_1 = theta_0(length(theta_0));

k = 1;
R_0 = (2^0.5)* k * (1 - cos((pi/4) + theta_0)).^0.5;
R_1 = (2^0.5)* k * (1 - cos((pi/4) + theta_1)).^0.5;

v_motor = -(R_0 - R_1)/delta_t;

plot(theta_0,v_motor);


