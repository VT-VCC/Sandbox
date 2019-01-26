function BL=rllt2BL(R,lat,long,to,time0,Re)  
%to is array of time increments in seconds
%time0 is start time of sim in datenum

tod = to/86400;
t = tod + time0;

%Finds Mag-Field Vector @ given alt, lat, long, time. 

BR    = R-Re; 
BLat  = 90-lat; 
BLong = long; 

for ndx=1:length(R)     
    BLi(ndx,:) = igrf(t(ndx),BLat(ndx),BLong(ndx),BR(ndx),'geodetic'); 
end  

BL=(1e-9)*BLi; %Mag field (Teslas)  