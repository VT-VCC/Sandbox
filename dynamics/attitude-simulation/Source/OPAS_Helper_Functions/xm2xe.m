function xe=xm2xe(mfv,lat,long)  

%Converts from  Mag-Field Frame to Earth Frame  

um = mfv(:,1); 
vm = mfv(:,2); 
wm = mfv(:,3);  

xm = [um';vm';wm'];  

lt  = pi/180*lat; 
lg = pi/180*long;  

for ndx=1:length(mfv)     
    xe(:,ndx)=[-sin(lt(ndx))*cos(lg(ndx)),-sin(lg(ndx)),cos(lt(ndx))*cos(lg(ndx));      
        -sin(lt(ndx))*sin(lg(ndx)), cos(lg(ndx)),cos(lt(ndx))*sin(lg(ndx));                
        cos(lt(ndx)),0,-sin(lt(ndx))]*xm(:,ndx); 
end  

xe=xe'; 