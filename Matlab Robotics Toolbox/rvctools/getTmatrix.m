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
%T06 = Fanuc_200id.fkine(q);