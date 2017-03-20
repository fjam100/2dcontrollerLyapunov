%% Check for ND matrix
cx=1;
cy=2;
theta=(0:0.01:2*pi)
for i=1:length(theta)
    
    res(i)=-cx*sin(theta(i)) - cy*sin(theta(i)) - sqrt(-4*cx*cy + ...
        (cx*sin(theta(i)) + cy* sin(theta(i))^2));
end