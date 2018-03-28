% Fanuc Model in Robotics Toolbox
clear all % Clear all varibles in the workspace
close all % Close all open figures
clc % Clear the command window

% Run the startup file of the Robotics Toolbox
startup_rvc;

disp('NOTE: For DH Table, all angles are in Radians, all lengths are in meters.')

%Constitute the Links filling in the missing parts using
%the DH-table of the robot:
syms pi

L(1) = Link('d', 0.254, 'a', 0, 'alpha', pi/2);
L(2) = Link('d', 0, 'a', 0.254, 'alpha', 0);
L(3) = Link('d', 0, 'a', 0.254, 'alpha', 0);
L(4) = Link('d', 0, 'a', 0, 'alpha', pi/2);
L(5) = Link('d', 0.0508, 'a', 0, 'alpha', 0);

% Define the joint angles symbolically:
syms th1 th2 th3 th4 th5 real
q = [th1, th2, th3, th4, th5]; 

%Form the robot
Fanuc_200id = SerialLink(L, 'name', '200id');

% Extract the Ti-1,i homogeneous transformation matrices
T00 = Fanuc_200id.base; % The base frame
T01 = L(1).A(q(1)); % The link#1 frame
T12 = L(2).A(q(2)); % The link#2 frame
T23 = L(3).A(q(3)); % The link#3 frame
T34 = L(4).A(q(4)); % The link#4 frame
T45 = L(5).A(q(5)); % The link#5 frame
T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
%Final transformation matrix
T05 = T04*T45;

%We can also find the final transformation using 'fkine' function:
T06 = Fanuc_200id.fkine(q)


%Find the end-effector position:
disp('End-effector x position:')
pretty(simplify(T06(1,4)))
disp('End-effector y position:')
pretty(simplify(T06(2,4)))
disp('End-effector z position:')
pretty(simplify(T06(3,4)))

%% Inverse Kin


