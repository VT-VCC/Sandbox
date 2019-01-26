function xi=xe2xi(xe,t,dti,wei,veo)

%Converts from Earth-Fixed Frame to Inertial Frame  

xe=xe';  

off=veo*3600*wei;

for ndx=1:length(xe)     
    xi(:,ndx) = [cos((wei)*(t(ndx)+dti)+off),-sin((wei)*(t(ndx)+dti)+off),0;                  
        sin((wei)*(t(ndx)+dti)+off), cos((wei)*(t(ndx)+dti)+off),0;                                      
        0,0,1]*xe(:,ndx);
end  

xi=xi';  