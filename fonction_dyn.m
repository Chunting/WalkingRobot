function [ A,H ] = fonction_dyn( q1,q2,dq1,dq2,theta )

%Methodology:
%
%Etape 1: position
%---------------Position of G1------------------
% x_g1=(l-s)*sin(q1);
% y_g1=(l-s)*cos(q1);
% ---------------Position of G2------------------
% x_g2=(l*sin(q1))+(s*sin(q1+q2));
% y_g2=(l*cos(q1))+(s*cos(q1+q2));
%
%Etape 2: velocities
% ----------------angular-----------------------
%       omega_1=dq1;     
%       omega_2=dq1+dq2; 
% -----------------linear-----------------------
%       V1=d(G1)/dt
%       V2=d(G2)/dt
%
%Etape 3: Energies
%   E1=1/2*(m*V1'*V1+omega_1'*I*omega_1);
%   E2=1/2*(m*V2'*V2+omega_2'*I*omega_2);
%   Total E= E1 + E2 = 1/2* Q' *A * Q
%   with Q=[dq1 dq2];

global S m L I g

A=[m*(L^2+S^2+2*L*S*cos(q2))+2*I+m*(L-S)^2,m*(S^2+L*S*cos(q2)+I);
   m*(S^2+L*S*cos(q2))+I,m*S^2+I];
%----------------Computation of H----------------

B=[-2*m*L*S*sin(q2);0];

C=[0 -m*L*S*sin(q2);
   m*L*S*sin(q2),0];

Q=[-m*g*((2*L-S)*sin( q1- theta) + S*sin(q1+q2 - theta));
   m*g*S*sin(theta-q1-q2)];

H=B*dq1*dq2+C*[dq1^2;dq2^2]+Q;

end

