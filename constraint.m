function [g_ineq,geq] = constraint(X)

q10=X(1);
dq2=X(2);
dq1=X(3);
Tf=X(4);

global q1f q2f q1m q2m d ti L tm

q20=pi-2*q10;

Zf = [q10;q20;dq1;dq2];

%==============IMPACT MODEL======================
[ A1,Jr2_t ] = fonction_impact(Zf(1),Zf(2));

M1=[A1 ,    -Jr2_t;
   Jr2_t' ,zeros(2)];

M2=[A1;zeros(2,4)];

xfd=-L*Zf(3)*cos(Zf(1));
zfd=-L*Zf(3)*sin(Zf(1));

Zf_p=inv(M1)*M2*[Zf(3);Zf(4);xfd;zfd];
Ix=Zf_p(3);
Iy=Zf_p(4);
%==============NEW CONTACT========================
q10d=Zf_p(1)+Zf_p(2);
q20d=-Zf_p(2);


%=============TRAJECTORY==========================

%parameters
t=0:Tf/50:Tf;

ParamT=[ti^4,ti^3,ti^2,ti,1;%<---qi
       4*ti^3,3*ti^2,2*ti,1,0;%<---d_qi
       Tf^4,Tf^3,Tf^2,Tf,1;%<---qf
       4*Tf^3,3*Tf^2,2*Tf,1,0;%<----d_qf
       tm^4,tm^3,tm^2,tm,1];%<--- middle point
   
Param_q1=ParamT\[q10;q10d;q1f;0;q1m];
Param_q2=ParamT\[q20;q20d;q2f;0;q2m];

a0=Param_q1(1);a1=Param_q1(2);a2=Param_q1(3);a3=Param_q1(4);a4=Param_q1(5);
b0=Param_q2(1);b1=Param_q2(2);b2=Param_q2(3);b3=Param_q2(4);b4=Param_q2(5);

q1=a0*t.^4+a1*t.^3+a2*t.^2+a3*t+a4;
dq1=4*a0*t.^3+3*a1*t.^2+2*a2*t+a3;
ddq1=12*a0*t.^2+6*a1*t+2*a2;

q2=b0*t.^4+b1*t.^3+b2*t.^2+b3*t+b4;
dq2=4*b0*t.^3+3*b1*t.^2+2*b2*t+b3;
ddq2=12*b0*t.^2+6*b1*t+2*b2;

Q=[q1.',q2.',dq1.',dq2.',ddq1.',ddq2.'];

dq=[dq1,dq2];

%==========OBJECTIF=====================================================
[~,R,Torque]=ss_passif(Q);


% % for constrains for arrqies, we only need to find the boundaries of them
% % 1st constains is for d
% g1 = 0.2 - d;
% 
% % 2nd constrains is for Ry
% g2 =  -R(:,2);
% 
% % 3rd constrain is for I2y
% g3= -Iy;
% 
% % 4th constrain for R
% g4= abs(R(:,1)./R(:,2))-0.7;
% 
% % 5th constrain is for Ix Iy
% g5= abs(Ix/Iy)-0.7;
% 
% % 6th constrain is for dq
% % joint 1
% g6= abs(dq(:,1))- 3;
% % joint 2
% g7= abs(dq(:,2))- 3;
% 
% %7th constrain is for torque
% % for torque 1
% g8= abs(Torque(:,1)) - 50;
% % for torque 2
% g9= abs(Torque(:,2)) - 50;
% 
% g10= -Iy;


g1 = 0.2 - d;

% 2nd constrains is for Ry
g2 =  max( -R(:,2) );

% 3rd constrain is for I2y
g3= -Iy;

% 4th constrain for R
g4= max( abs(R(:,1)./R(:,2)) )-0.7;

% 5th constrain is for Ix Iy
g5= abs(Ix/Iy)-0.7;

% 6th constrain is for dq
% joint 1
g6= max(abs(dq(:,1)))- 3;
% joint 2
g7= max(abs(dq(:,2)))- 3;

%7th constrain is for torque
% for torque 1
g8= max(abs(Torque(:,1))) - 50;
% for torque 2
g9= max(abs(Torque(:,2))) - 50;


g_ineq=[g1;g2;g3;g4;g5;g6;g7;g8;g9];

geq=[];
end

