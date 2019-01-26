function [galo,gala]=gai2gall(gai,t,dti,wie,i,veo)  

%Creates Plottable Matrices of the Lat and Long defining 
%the Ground Area that can view the S/C from the coordinates 
%of that boundary in the inertial frame  

ui = gai(:,1); 
vi = gai(:,2); 
wi = gai(:,3);  

f=length(t); 
off=veo*3600*wie; 
for ndx = 1:length(gai)     
    ue(ndx) =  cos((wie)*(t(f)+dti)+off)*ui(ndx)+sin((wie)*(t(f)+dti)+off)*vi(ndx);     
    ve(ndx) = sin((wie)*(t(f)+dti)+off)*ui(ndx)+cos((wie)*(t(f)+dti)+off)*vi(ndx);     
    we(ndx) = wi(ndx); 
end  

gae = [ue',ve',we']; 

gall = xe2rll(gae);  

long = gall(:,3); 
lat  = gall(:,2); 

count=0;  

for index=1:length(long)     
    if (long(index)<-179)|(long(index)>179)         
        count=count+1;     
    end
end

if count>0     
    n = 1;     
    m = 0;     
    for ndx=1:length(long)         
        if i(length(i))>(pi/2)             
            if ndx==1|ndx==2                 
                longm(ndx,n)=long(ndx);                 
                latm(ndx,n)=lat(ndx);             
            else
                if (long(ndx)>long(ndx-1))|((long(ndx)<long(ndx-1))&(long(ndx1)>long(ndx-2)))                     
                    n=n+1;                     
                    m=ndx-1;
                    longm(ndx-m,n)=long(ndx);                     
                    latm(ndx-m,n)=lat(ndx);                 
                else
                    longm(ndx-m,n)=long(ndx);                     
                    latm(ndx-m,n)=lat(ndx);                 
                end
            end
        elseif i(length(i))<(pi/2)             
            if ndx==1|ndx==2                 
                longm(ndx,n)=long(ndx);                 
                latm(ndx,n)=lat(ndx);             
            else
                if (long(ndx)<long(ndx-1))|((long(ndx)>long(ndx-1))&(long(ndx1)<long(ndx-2)))                     
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
    galo=longm;     
    gala=latm; 
else
    galo=long;     
    gala=lat; 
end