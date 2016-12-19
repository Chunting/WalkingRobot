clear all
clc

%% 1. we define the initial point for optimization searching
% [q10, dq1f, dq2f, T]
% q10 is the initial position for the joint 1;
% dq1f and dq2f are the final velocities for the joint 1 and 2.
% T means the transfer time.
X = [1,0.1,0.1, 5]; 

global Tf
Tf=X(4); % define Tf for the function being called

%% 2. we define the robot parameters
global theta m g q1f q2f q1m q2m d L ti tm I S

% information about the environment
I = 0.08;
S = 0.45;
L=0.8;
d=L;
theta=0;
m=2;
g=9.81;

% expected trajectory characteristics
ti=0;	% beginning time 
q1f=1;	% final velocities of joint 1;
q2f=1;	% final velocities of joint 2;

tm=X(4)/2; % time of crossing intermediate configuration
q1m=0.5;  % joint 1 of intermediate configuration	
q2m=0.5;  % joint 2 of  crossing intermediate configuration


%% 3. we define the boundaries for the optimization parameters
%decision vars
% lower bound
lb = [-3,-3,-3,0];
% upper bound
ub = [3,3,3,100];

%% 3. Run the optimization program
% objective function is defined as objfun.m
% constrain function is built in constraint.m

% options of optimization
	nbIteration=500;
	nbEval=1000;
% OPTIMIZATION: obj function -> 'resol', constraints -> 'mycon'
options=optimset('Display','iter','TolX',1e-6,'TolFun',1e-8,'MaxIter',nbIteration,'MaxFunEvals',nbEval);

% call the solver
[Jsolcons,Fval,EXITFLAG] = fmincon('objfun',X,[],[],[],[],lb,ub,'constraint',options);