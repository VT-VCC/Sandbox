function xi=xo2xi(r,nu,om,Om,i)  

%Converts from Orbit Frame to Inertial Frame in Orbit Propagator 
uo = r.*cos(nu+om); 
vo = r.*sin(nu+om); 
ui = uo.*cos(Om)-vo.*cos(i).*sin(Om); 
vi = uo.*sin(Om)+vo.*cos(i).*cos(Om);
wi = vo.*sin(i);  
xi = [ui,vi,wi];  