function gai=r2gai(xi,Re,Lv)

%Creates the boundary on Earth in inertial coordinates that can view the S/C  

Rv = xi(length(xi),:); 
Rm = norm(Rv);  

r  = Re*cos(Lv)*(cos(asin(Re/Rm*cos(Lv)))-Re/Rm*sin(Lv)); 
cm = Re*(sin(Lv)*cos(asin(Re/Rm*cos(Lv)))+Re/Rm*(cos(Lv))^2); 
cv = cm/Rm*Rv; 

ci1 = (-r):10:(+r); 
ci2 = ci1; 
ci  = [flipud(ci1');ci2';ci1(length(ci1))];
cj1 =  sqrt(r^2-(ci1).^2); 
cj2 = -sqrt(r^2-(ci2).^2); 
cj  = [flipud(cj1');cj2';cj1(length(cj1))]; 
ck  = zeros(size(ci)); 
cx  = [ci,cj,ck]; 

if Rv(1)>0     
    th  = atan(Rv(2)/Rv(1))+pi/2; 
else
    th  = atan(Rv(2)/Rv(1))+3*pi/2; 
end

ph  = pi-atan(sqrt((Rv(1))^2+(Rv(2))^2)/Rv(3));      

cii = (ci*cos(th)-cj*cos(ph)*sin(th)-ck*sin(ph)*sin(th))+cv(1); 
cji = (ci*sin(th)+cj*cos(ph)*cos(th)+ck*sin(ph)*sin(th))+cv(2); 
cki = (          -cj*sin(ph)        +ck*cos(ph))+cv(3); 
cxi = [cii,cji,cki]; 
gai = cxi;  