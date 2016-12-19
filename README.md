# WalkingRobot
This codes is for the TP 3 of the walking robot given by the professor yanick austin in ECN.
The problem statement is given by the Texte_TP_optim.pdf

# Codes introductions
This codes provides a optimization method for a trajectory generation for a compass robot.

 1. optimization variables are [q10, dq1f, dq2f, T]
	q10 is the initial position for the joint 1;
	dq1f and dq2f are the final velocities for the joint 1 and 2.
	T means the transfer time.
2. constrains is represented in constrain.m
3. object is expressed in objfun.m.

# How to run the program
1. run the Opt.m (with X = [1,0.1,0.1, 5] by default)
2. results including X and g_ineg are shown in Matlab.
