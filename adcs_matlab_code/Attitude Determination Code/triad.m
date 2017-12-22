function [Rbi]=triad(v1b,v2b,v1i,v2i)

t1b=v1b;
t2b=(cross(v1b,v2b))/norm(cross(v1b,v2b));
t3b=cross(t1b,t2b);

t1i=v1i;
t2i=(cross(v1i,v2i))/norm(cross(v1i,v2i));
t3i=cross(t1i,t2i);

a=size(t1b);
if a(1)==1
    Rbt=[t1b',t2b',t3b'];
    Rit=[t1i',t2i',t3i'];
else
    Rbt=[t1b,t2b,t3b];
    Rit=[t1i,t2i,t3i];
end
Rbi=Rbt*Rit';
end
