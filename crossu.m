function c = crossu( a, b )
c = zeros(3, 1);
c(1) = a(2) * b(3) - a(3) * b(2);
c(2) = a(3) * b(1) - a(1) * b(3);
c(3) = a(1) * b(2) - a(2) * b(1);
cmag = sqrt(c(1)^2 + c(2)^2 + c(3)^2);
for i = 1:3
    c(i) = c(i) / cmag;
end
end