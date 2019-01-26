function ad=roundto(a,b)  

% Rounds a number (a) to a resolution (b) 
c = rem(a,b); 
d = c/b; 
[m,n]=size(a); 

for mdx=1:m     
    for ndx=1:n         
        if d(mdx,ndx)>=0.5             
            a(mdx,ndx)=a(mdx,ndx)+(b-c(mdx,ndx));         
        elseif d(mdx,ndx)<=-0.5             
            a(mdx,ndx)=a(mdx,ndx)-(b+c(mdx,ndx));         
        else
            a(mdx,ndx)=a(mdx,ndx)-c(mdx,ndx);         
        end
    end
end

ad=a;  