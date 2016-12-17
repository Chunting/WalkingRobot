clear all
clc

X = [1,1,1,1]; 

global theta m g q1f q2f q1m q2m d L ti tm I S Tf

Tf=X(4);

I = 0.08;
S = 0.45;

q1f=1;
q2f=1;
q1m=0.5;
q2m=0.5;

ti=0;
tm=X(4)/2;

L=0.8;
d=L;
theta=0;
m=2;
g=9.81;



%decision vars
% lower bound
lb = [-2,-2,-2,0];
% upper bound
ub = [2,2,2,100];

nbIteration=500;
nbEval=1000;

% OPTIMIZATION: obj function -> 'resol', constraints -> 'mycon'
options=optimset('Display','iter','TolX',1e-6,'TolFun',1e-8,'MaxIter',nbIteration,'MaxFunEvals',nbEval);
[Jsolcons,Fval,EXITFLAG] = fmincon('objfun',X,[],[],[],[],lb,ub,'constraint',options);