function [g_ineq,geq] = constraint(X)

% q10=X(1);
% dq2=X(2);
% dq1=X(3);
% Tf=X(4);
%==========1. Parameters=====================================================
 global q1f q2f q1m q2m d ti L tm
 global Q;
 % Q=[q1.',q2.',dq1.',dq2.',ddq1.',ddq2.'];
 dq1 = Q(:,3);
 dq2 = Q(:,4);
 global Zf_p;
% output Zf_p=[dotq1_af, dotq2_af, dotx_af, doty_af, I_rx, I_ry];

%==========OBJECTIF=====================================================
% R(Ts, 2) is the reaction force
% Torque(Ts, 2) is the torques
[~,R,Torque]=ss_passif(Q);

g1 = 0.2 - d;

% 2nd constrains is for Ry reaction force along y
g2 =  max( -R(:,2) );

% 3rd constrain is for I2y
Iy =  Zf_p(6);
g3= -Iy;

% 4th constrain for R
g4= max( abs(	R(:,1)./R(:,2)	) )-0.7;

% 5th constrain is for Ix Iy
Ix = Zf_p(5);
g5= abs(Ix/Iy)-0.7;

% 6th constrain is for dq
% joint 1
g6= max(abs(dq1))- 3;
% joint 2
g7= max(abs(dq2))- 3;

%7th constrain is for torque
% for torque 1
g8= max(abs(Torque(:,1))) - 50;
% for torque 2
g9= max(abs(Torque(:,2))) - 50;


g_ineq=[g1;g2;g3;g4;g5;g6;g7;g8;g9]

geq=[];

end

