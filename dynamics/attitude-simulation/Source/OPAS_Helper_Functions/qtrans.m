function qt = qtrans(q1,q2)  
% Returns sum of two successive quaternion rotations 
q = [     
    q2(4)   q2(3)  -q2(2)   q2(1);    
    -q2(3)   q2(4)   q2(1)   q2(2);     
    q2(2)  -q2(1)   q2(4)   q2(3);    
    -q2(1)  -q2(2)  -q2(3)   q2(4) 
    ];  

qt = (q*[q1(1);q1(2);q1(3);q1(4)])';  