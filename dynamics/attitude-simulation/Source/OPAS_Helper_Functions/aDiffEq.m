function stateD = aDiffEq(t,state,I,K,mub,to,Bi,Treq,test,mm)  

%The Attitude Differential Equations  

w=state(1:3); 
q=state(4:7);  

Bb = Bi2Bb(to,Bi,t,q');         %Magnetic field in body frame (Tesla)  

if test==1     
    T = Treq;                   %Torque directly from command (for debugging) 
else
    T = cross(mub',Bb');        %Torque using magnetorquers 
end

mm1 = interp1(to,mm,t);         %Mean Motion(rad/s)  

C11 = 1-2*(q(2)^2+q(3)^2);      %3 components of quaternion transformation 
C21 = 2*(q(1)*q(2)-q(3)*q(4));  %matrix necessary for gravity gradient 
C31 = 2*(q(1)*q(3)+q(2)*q(4)); 

% GG = [0 0 0]; 
GG = [-K(1)*3*mm1^2*C21*C31 -K(2)*3*mm1^2*C11*C31 -K(3)*3*mm1^2*C11*C21];  

wD = [K(1)*w(2)*w(3)+T(1)/I(1)+GG(1);
      K(2)*w(3)*w(1)+T(2)/I(2)+GG(2);       
      K(3)*w(1)*w(2)+T(3)/I(3)+GG(3)];  
  
Wp = [    0,  w(3), -w(2),  w(1);       
      -w(3),     0,  w(1),  w(2);        
       w(2), -w(1),     0,  w(3);       
       -w(1), -w(2), -w(3),     0]; 
   
     qD = 1/2*Wp*q;  
     
stateD=[wD;qD];  