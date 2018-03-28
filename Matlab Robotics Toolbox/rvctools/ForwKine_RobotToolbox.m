% Fanuc Model in Robotics Toolbox
clear all % Clear all varibles in the workspace
close all % Close all open figures
clc % Clear the command window

% Run the startup file of the Robotics Toolbox
startup_rvc;

disp('NOTE: For DH Table, all angles are in Radians, all lengths are in meters.')

%Constitute the Links filling in the missing parts using
%the DH-table of the robot:

L(1) = Link('d', 0, 'a', 0, 'alpha', deg2rad(90));
L(2) = Link('d', 0, 'a', 0.254, 'alpha', 0);
L(3) = Link('d', 0, 'a', 0.254, 'alpha', 0);
L(4) = Link('d', 0, 'a', 0, 'alpha', deg2rad(90));
L(5) = Link('d', 0.0508, 'a', 0, 'alpha', 0);


%t1=0;
%t2=65.8;
%t3=-73.3;
%t4=73.3;
%t5=0;


theta1=t1;
theta2=t2;
theta3=t2+t3;
theta4=t2-theta3;
theta5=0;

% Set joint angles
q = [theta1, theta2-90, theta3-86, theta4+86, theta5]*pi/180; 

syms theta1 theta2 theta3 theta4 theta5
qsym = [theta1, theta2-90, theta3-86, theta4+86, theta5]*pi/180;


%% PLOT

%Form the robot
Fanuc_200id = SerialLink(L, 'name', '200id');
% Print the DH table and other robot properties
Fanuc_200id

% Plot the robot using the Robotics Toolbox
Fanuc_200id.plot(q);
title(['The Fanuc 200id Robot'])
% ext = 20;
% xlim([-ext,ext]);ylim([-ext,ext]);zlim([0,ext+10]); % Set x/y/z plot limits
hold on
str = {['q=[',num2str(q(1)*180/pi,'%3.0f'),'°,',num2str(q(2)*180/pi,'%3.0f'),'°, ',...
num2str(q(3)*180/pi,'%3.0f'),'°, ',num2str(q(4)*180/pi,'%3.0f'),'°,',num2str(q(5)*180/pi,'%3.0f'),'°]']};
annotation('textbox',[0.0 0.1 0.3 .1],'string',str,'FitBoxToText','on','BackgroundColor','w')
hold off



% Extract the Ti-1,i homogeneous transformation matrices
T00 = Fanuc_200id.base; % The base frame
T01 = L(1).A(q(1)); % The link#1 frame
T12 = L(2).A(q(2)); % The link#2 frame
T23 = L(3).A(q(3)); % The link#3 frame
T34 = L(4).A(q(4)); % The link#2 frame
T45 = L(5).A(q(5)); % The link#3 frame
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;

%% syms

T01s = L(1).A(qsym(1)); % The link#1 frame
T12s = L(2).A(qsym(2)); % The link#2 frame
T23s = L(3).A(qsym(3)); % The link#3 frame
T34s = L(4).A(qsym(4)); % The link#2 frame
T45s = L(5).A(qsym(5)); % The link#3 frame
T02s = T01s*T12s;
T03s = T02s*T23s;
T04s = T03s*T34s;
T05s = T04s*T45s;

%%



disp('Forward kinematics to the tool frame:')
T06 = Fanuc_200id.fkine(q)

disp('Euler angles for the given rotation matrix:')
eul = tr2eul(T06)

disp('Roll-Pitch-Yaw angles for the given rotation matrix:')
rpy = tr2rpy(T06)

disp('Jacobian wrt the given configuration:')
jn = Fanuc_200id.jacobn(q)


disp('Inverse kinematics for the given configuration:')
mask = [1 1 1 1 1 0];
q_init = [0, -10, 40, 35, 65]*pi/180;
Q = Fanuc_200id.ikine(T06,q_init,mask,'alpha',1,'pinv')*180/pi
 


