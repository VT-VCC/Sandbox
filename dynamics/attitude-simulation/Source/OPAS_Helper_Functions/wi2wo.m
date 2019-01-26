function wo = wi2wo(w,q,mm)  

%Converts angular rates from inertial to orbital 
T = [     
    1-2*(q(2)^2+q(3)^2)       2*(q(1)*q(2)+q(3)*q(4))   2*(q(1)*q(3)-q(2)*q(4));     
    2*(q(1)*q(2)-q(3)*q(4))       1-2*(q(1)^2+q(3)^2)   2*(q(2)*q(3)+q(1)*q(4));     
    2*(q(1)*q(3)+q(2)*q(4))   2*(q(2)*q(3)-q(1)*q(4))       12*(q(1)^2+q(2)^2) 
    ];  

wo = ([w(1);w(2);w(3)]-T*[0;0;mm])';  