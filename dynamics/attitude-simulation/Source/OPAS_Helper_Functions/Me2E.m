function E=Me2E(M,e)  

%Solves for Eccentric Anomaly given Mean Anomaly and 
%Eccentricity using Newton's Iteration Method  

Ec = M; 
Ei = Ec - (Ec-e*sin(Ec)-M)/(1-e*cos(Ec));  

while (abs(Ei-Ec)>1e-7)     
    Ec = Ei;     
    Ei = Ec - (Ec-e*sin(Ec)-M)/(1-e*cos(Ec)); 
end;  

E = Ei;  