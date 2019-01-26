function qe = qerr(qd,qa)   
%Calculates the quaternion error rotation 
%qd = desired, qa = actual 
q = [     
    qd(4)   qd(3)  -qd(2)  -qd(1);
    -qd(3)   qd(4)   qd(1)  -qd(2);
    qd(2)  -qd(1)   qd(4)  -qd(3);     
    qd(1)   qd(2)   qd(3)   qd(4) 
    ];

qe = (q*[qa(1);qa(2);qa(3);qa(4)])';