function vsc = tosc(A, v123)

vsc = zeros(3, 1);
for i = 1:3
    for j = 1:3
        vsc(i) = vsc(i) * A(j, i) * v123(j);
    end
end
vmag = sqrt(vsc(1)^2 + vsc(2)^2 + vsc(3)^2);
for k = 1:3
    vsc(k) = vsc(k) / vmag;
end
end