function keps=tle2keps(tle,mu)  

%Converts NASA TLE to Keplerian Elements 

i  = tle(1); 
Om = tle(2); 
e  = tle(3); 
om = tle(4); 
M  = tle(5); 
n  = tle(6); 

a  = (mu/n^2)^(1/3);  

E  = Me2E(M,e); 
nu = 2*atan(sqrt((1+e)/(1-e))*tan(E/2));  

keps = [a; e; nu; i; Om; om]; 