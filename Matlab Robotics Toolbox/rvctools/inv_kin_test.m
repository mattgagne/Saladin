close all; clear all; clc;
syms x y z
x = 0.3; y = 0.1; z = .1;

digits(3);
% 2D problem
Xb = sqrt(x^2 + y^2);
Yb = z - 0.254;
L1 = 0.254;
L2 = 0.254;

D = (Xb^2 + Yb^2 - L1^2 - L2^2)/(2*L1*L2);

q2 = atan2(-sqrt(1-D^2), D);
q1 = atan2(Yb,Xb) - (-acos((Xb^2 + Yb^2 + L1^2 - L2^2)/(2*L1*sqrt(Xb^2+Yb^2))));

% Back to full problem
th1 = vpa(atan2(y,x))
th2 = vpa(q1)
th3 = vpa(q2)
th4 = vpa(q1 + q2)

% test in foward kin
%th1 = 0; th2 = 0; th3 = 0; th4 = 0;
x_ = (127*cos(th1)*cos(th2))/500 - (127*cos(th1)*sin(th2)*sin(th3))/500 + (127*cos(th1)*cos(th2)*cos(th3))/500;
y_ = (127*cos(th2)*sin(th1))/500 - (127*sin(th1)*sin(th2)*sin(th3))/500 + (127*cos(th2)*cos(th3)*sin(th1))/500;
z_ = (127*sin(th2))/500 + (127*cos(th2)*sin(th3))/500 + (127*cos(th3)*sin(th2))/500 + 127/500;