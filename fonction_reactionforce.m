function F = fonction_reactionforce(q1,q2,dq1,dq2, ddq1, ddq2)

global L S m g;

%% get the theta
global theta;
    
%% obtain the ddxG
    ddXg_1 = ddq1 * (S-2*L) * cos(q1) - ddq1^2 * (S-2*L) * sin(q1) ...
-(ddq1+ddq2) * S * cos(q1+q2) + (dq1 + dq2)^2*S*sin(q1+q2);
    ddXg_2 = -ddq1 * (2*L-S) * sin(q1) - ddq1^2 * (2*L-2*S) * cos(q1)...
-(ddq1+ddq2) * S * sin(q1+q2) - (dq1 + dq2)^2*S*cos(q1+q2);
    %
    ddXg = [ddXg_1;ddXg_2 ]/2;
%% get the F
    F = m * (ddXg-g*[sin(theta);-cos(theta)]);
end