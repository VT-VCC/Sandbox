function [longm,latm]=col2mat(long,lat,i)

%Converts from Column to Matrix 
%           1 
%           2        1 -3 
%Changes   -3   to   2 -2   
%          -2        0 -1   
%          -1  

n = 1; 
m = 0;

for ndx=1:length(long)     
    if i(ndx)>(pi/2)         
        if ndx==1             
            longm(ndx,n)=long(ndx);             
            latm(ndx,n)=lat(ndx);         
        else
            if long(ndx)>long(ndx-1)                 
                n=n+1;                 
                m=ndx-1;                 
                longm(ndx-m,n)=long(ndx);                 
                latm(ndx-m,n)=lat(ndx);             
            else
                longm(ndx-m,n)=long(ndx);                 
                latm(ndx-m,n)=lat(ndx);             
            end
        end
    elseif i(ndx)<(pi/2)         
        if ndx==1             
            longm(ndx,n)=long(ndx);             
            latm(ndx,n)=lat(ndx);         
        else
            if long(ndx)<long(ndx-1)                 
                n=n+1;                 
                m=ndx-1;                 
                longm(ndx-m,n)=long(ndx);                 
                latm(ndx-m,n)=lat(ndx);             
            else
                longm(ndx-m,n)=long(ndx);                 
                latm(ndx-m,n)=lat(ndx);             
            end
        end
    else
        longm=long;         
        latm=lat;     
    end
end