function rll=xe2rll(xe)  

%Converts from Earth Frame to Radius, Longitude, & Latitude  

ue = xe(:,1); 
ve = xe(:,2); 
we = xe(:,3);

for ndx = 1:length(xe)     
    r(ndx)    = sqrt((ue(ndx))^2+(ve(ndx))^2+(we(ndx))^2);     
    lat(ndx)=asin(we(ndx)/r(ndx))*180/pi;     
    
    long(ndx)=(ve(ndx)/abs(ve(ndx)))*acos(ue(ndx)/sqrt((ue(ndx))^2+(ve(ndx))^2))*180/pi; 
end  

rll = [r',lat',long'];