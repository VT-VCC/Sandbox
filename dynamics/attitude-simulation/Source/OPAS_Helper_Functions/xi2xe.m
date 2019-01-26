function xe=xi2xe(xi,t,dti,wie,veo)  

%Converts from Inertial Frame to Earth Frame 

ui = xi(:,1); 
vi = xi(:,2); 
wi = xi(:,3);

off=veo*3600*wie;  

for ndx = 1:length(xi)     
    ue(ndx) =  cos((wie)*(t(ndx)+dti)+off)*ui(ndx)+sin((wie)*(t(ndx)+dti)+off)*vi(ndx);    
    ve(ndx) = sin((wie)*(t(ndx)+dti)+off)*ui(ndx)+cos((wie)*(t(ndx)+dti)+off)*vi(ndx);     
    we(ndx) = wi(ndx); end  

xe = [ue',ve',we']; 