# WalkingRobot
This codes is for the TP 3 of the walking robot given by the professor yanick austin in ECN.
The problem statement is given by the Texte_TP_optim.pdf

# Codes introductions
This codes provides a optimization method for a trajectory generation for a compass robot.

The optimization variables are [q10, dq10, dq20, T]
q10 is the initial position for the joint 1;
dq10 and dq20 are the initial velocities for the joint 1 and 2.
T means the transfer time.

Parameters known.

geometry information about the robot and environment
global theta m g I S d L 

expected trajectory coefficients
global q1f q2f q1m q2m ti tm
q1f and q2f are final velocities of the robot;
ti is the beginning time
q1m,  q2m and ti refer to the intermediate configuration and the corresponding time

# How to run the program
1. main program is developed in Opt.m
2. 
