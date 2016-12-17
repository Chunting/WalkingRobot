function [ A1,Jr2_t ] = fonction_impact( q1,q2 )

global m S I L

%=========================================
%from the kinetic energy without implicit 
%constraint:
%---------------New Position of G1------------------
%   x_G1=x+S*sin(q1)
%   y_G1=y-S*cos(q1)
%
%---------------New Position of G2------------------
%   x_G2=x-S*sin(q1+q2)
%   y_G2=y+S*cos(q1+q2)
%
%P=[dq1 dq2 dx dy]'
%then Ec=Ec1+Ec2=1/2 *P'*A1*P
%
%For the comutation of this part we used simbolics.m

%diagonal
A11=2*(m*S^2+I);			
A22=m*S^2+I;
A33=2*m;
A44=A33;

%right upper 
A12=  m*S^2+I;	
A13=  m*S*( cos(q1) - cos(q1+q2) );	
A14=  m*S*( sin(q1) - sin(q1+q2) );
A23= -m*S*cos(q1+q2);	
A24= -m*S*sin(q1+q2);	
A34=  0;

%left lower
A21=A12;
A31=A13;
A41=A14;
A32=A23;
A42=A24;
A43=A34;

%Matrix A1
A1=[A11,A12,A13,A14;
    A21,A22,A23,A24;
    A31,A32,A33,A34;
    A41,A42,A43,A44];

%======================================
%From contact conditions after impact
%Position of the contact point:
%       x_p=x-L*sin(q1+q2)
%       y_p=y+L*cos(q1+q2)
%
%     ---> Jr2*[dX;dq]=0

%Jacobian Matrix
Jr2 = [-L * cos(q1+q2), -L * cos(q1+q2) 1 0;
       -L * sin(q1+q2), -L * sin(q1+q2) 0 1;];
 
%transpose of Jr2
Jr2_t=Jr2';


end

