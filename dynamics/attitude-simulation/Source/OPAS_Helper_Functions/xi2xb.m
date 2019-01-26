function xb=xi2xb(xi,q)  

%Inertial->Body Quaternion Transformation Matrix for Orbit Propagator  

xin=xi';  

for ndx=1:length(xi)     
    i2b=[1 - 2*(q(ndx,2)^2 + q(ndx,3)^2),   2*(q(ndx,1)*q(ndx,2) + q(ndx,3)*q(ndx,4)), 2*(q(ndx,1)*q(ndx,3) - q(ndx,2)*q(ndx,4)); 2*(q(ndx,1)*q(ndx,2) - q(ndx,3)*q(ndx,4)), 1 - 2*(q(ndx,1)^2 + q(ndx,3)^2),   2*(q(ndx,2)*q(ndx,3) + q(ndx,1)*q(ndx,4)); 2*(q(ndx,1)*q(ndx,3) + q(ndx,2)*q(ndx,4)), 2*(q(ndx,2)*q(ndx,3) - q(ndx,1)*q(ndx,4)), 1 - 2*(q(ndx,1)^2 + q(ndx,2)^2)];     
    xbn(:,ndx)= i2b*xin(:,ndx); 
end  

xb=xbn';  