function v123 = to123( A, vsc )

v123 = zeros(3, 1);
for j = 1:3
    for i = 1:3
        v123(j) = v123(j) + A(j, i) * vsc(i);
    end
end
vmag = sqrt(vmag(1)^2 + vmag(2)^2 + vmag(3)^2);
for k = 1:3
    v123(k) = v123(k) / vmag;
end
end