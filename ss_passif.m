function [J,R, T] = ss_passif( z )
 global theta Tf m g d;


 
 t=0:Tf/50:Tf;
 
 % the samling peroid of time
 Ts=length(t);
 
 % initisation of torque matrix
 T = zeros(Ts,2);
 
 %Jm=zeros(Ts,1);
 
 for i=1:Ts
     dq=[z(i,3);z(i,4)];
     ddq=[z(i,5);z(i,6)];
     [ A,H ] = fonction_dyn( z(i,1),z(i,2),z(i,3),z(i,4),theta );

     Torque=A*ddq+H.*dq; %%%%%%%%%%%
     
     T(i,:)=Torque.';% matrix stores Torques in trajectory
     
     Jm(i)=(Torque.'*Torque)* Tf/50;
     
     R(i,:)=fonction_reactionforce( z(i,1),z(i,2),z(i,3),z(i,4),z(i,5),z(i,6));     
 end
 
     J=(1/(m*g*d))* sum(Jm);
 
end

