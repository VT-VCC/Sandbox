function Bb=Bi2Bb(to,Bio,ta,q)  
%Inertial->Body Quaternion Transformation Matrix for Orbit Propagator      
Bia=interp1(to,Bio,ta); 
Bia=Bia';  

for ndx=1:length(ta)     
    i2b=[1 - 2*(q(ndx,2)^2 + q(ndx,3)^2),   2*(q(ndx,1)*q(ndx,2) + q(ndx,3)*q(ndx,4)), 2*(q(ndx,1)*q(ndx,3) - q(ndx,2)*q(ndx,4));
        2*(q(ndx,1)*q(ndx,2) - q(ndx,3)*q(ndx,4)), 1 - 2*(q(ndx,1)^2 + q(ndx,3)^2),   2*(q(ndx,2)*q(ndx,3) + q(ndx,1)*q(ndx,4));           
        2*(q(ndx,1)*q(ndx,3) + q(ndx,2)*q(ndx,4)), 2*(q(ndx,2)*q(ndx,3) - q(ndx,1)*q(ndx,4)), 1 - 2*(q(ndx,1)^2 + q(ndx,2)^2)];     
    Bb(:,ndx)= i2b*Bia(:,ndx); 
end  

Bb=Bb';  