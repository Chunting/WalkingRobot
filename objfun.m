function f = objfun( X )

%==============0. Optimisation variables======================	
% optimisation variables are q10, q1fd, q2fd, Tf
q10=X(1);
%--------------------------------- Robin modification-------------------------------- 
q1fd = X(2);   	% before dq2=X(2);
q2fd = X(3); 	% before dq1=X(3);
%---------------------------------Robin modification-------------------------------- 
Tf=X(4);

global q20 q1f q2f q1m q2m L ti tm



%==============1. Trajectory Generation Parameters======================
%---------------------1.1 initial and finial position 
% q10, q1f, q2f, q1m q2m  are known
q20=pi-2*q10;


%---------------------1.2. initial and final velocities---------------------
% 1) final velocities q1fd and q2fd are known

% 2) the initial velocities through impact model 

	% (1) states before impact
	% the joint velocities before impact
	Zf = [q1f;q2f;q1fd;q2fd];

	%---------------------------------
	xfd=-L*Zf(3)*cos(Zf(1));
	yfd=-L*Zf(3)*sin(Zf(1));
	%---------------------------------

	%(2) IMPACT MODEL
	% complete the model
	[ A1,Jr2_t ] = fonction_impact(Zf(1),Zf(2));

	M1=[A1 ,    -Jr2_t;
   	Jr2_t' ,zeros(2)];

	M2=[A1;zeros(2,4)];

	% apply the model
	% output Zf_p=[dotq1_af, dotq2_af, dotx_af, doty_af, I_rx, I_ry];
	Zf_p=inv(M1)*M2*[Zf(3);Zf(4);xfd;yfd];

	% initial velocities 
	q10d=Zf_p(1)+Zf_p(2);
	q20d=-Zf_p(2);




%=============2. TRAJECTORY Generation==========================

% partitioned time interval
t=0:Tf/50:Tf;

% solve linear matrix
ParamT=[ti^4,ti^3,ti^2,ti,1;%<---qi
       4*ti^3,3*ti^2,2*ti,1,0;%<---d_qi
       Tf^4,Tf^3,Tf^2,Tf,1;%<---qf
       4*Tf^3,3*Tf^2,2*Tf,1,0;%<----d_qf
       tm^4,tm^3,tm^2,tm,1];%<--- middle point
%--------------------------------- Robin modification--------------------------------    
Param_q1=ParamT\[q10;q10d;q1f;q1fd;q1m]; % before Param_q1=ParamT\[q10;q10d;q1f;0;q1m]
Param_q2=ParamT\[q20;q20d;q2f;q2fd;q2m]; % before Param_q2=ParamT\[q20;q20d;q2f;0;q2m]
%--------------------------------- Robin modification-------------------------------- 

% obtain trajectory 
a0=Param_q1(1);a1=Param_q1(2);a2=Param_q1(3);a3=Param_q1(4);a4=Param_q1(5);
b0=Param_q2(1);b1=Param_q2(2);b2=Param_q2(3);b3=Param_q2(4);b4=Param_q2(5);

% q1(t)
q1=a0*t.^4+a1*t.^3+a2*t.^2+a3*t+a4;
dq1=4*a0*t.^3+3*a1*t.^2+2*a2*t+a3;
ddq1=12*a0*t.^2+6*a1*t+2*a2;

% q2(t)
q2=b0*t.^4+b1*t.^3+b2*t.^2+b3*t+b4;
dq2=4*b0*t.^3+3*b1*t.^2+2*b2*t+b3;
ddq2=12*b0*t.^2+6*b1*t+2*b2;

Q=[q1.',q2.',dq1.',dq2.',ddq1.',ddq2.'];

%=============================3. OBJECTIF==================================
% the input is the Q=[q1.',q2.',dq1.',dq2.',ddq1.',ddq2.'] for the whole trajectory.
% the output f is the transfer cost
[f,~,~]=ss_passif(Q);


end

