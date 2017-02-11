function stateD=oDiffEq(t,state,mu,J2,nD20,Re) 

%The Orbital Differential Equations  

n   = state(1);%Mean Motion 
nD  = state(2);%First Derivative of Mean Motion (rad/s2) 
a   = state(3);%(mu/n^2)^(1/3);Semi-Major Axis 
e   = state(4);%Eccentricity 
i   = state(5);%Inclination (rad) 
Om  = state(6);%RAAN (rad) 
om  = state(7);%Arg of Perigee (rad) 
nu  = state(8);%True Anomaly (rad) 
nuD = state(9);%Angular Velocity (rad/s) 
r   = state(10);%(a*(1-e^2))/(1+e*cos(nu))%Radius (km) 
rD  = state(11);%Radial Velocity (km/s)

stateD(1)  = nD; 
stateD(2)  = nD20; 
stateD(3)  = 0; 
stateD(4)  = 0; 
stateD(5)  = 0; 
stateD(6)  = (pi*cos(i))/(43200*n)*(-0.00338-0.00154)*pi/15552000 ...             
    -1.5*n*J2*(Re/a)^2*cos(i)/((1-e^2)^2); 
stateD(7)  = (pi*(4-5*(sin(i))^2))/(43200*n)*(0.00169+0.00077)*pi/15552000 ...              
    +0.75*n*J2*(Re/a)^2*(4-5*(sin(i))^2)/((1-e^2)^2); 
stateD(8)  = nuD;                  
stateD(9)  = -2*rD*nuD/r;          
stateD(10) = rD;                  
stateD(11) = -mu/r^2 + r*nuD^2;

stateD=stateD';  