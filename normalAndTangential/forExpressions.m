syms theta xr yr r0 x y kt kn xc yc
epsilonn=dot([xc+r0*cos(theta);yc+r0*sin(theta)]-[x;y],[-cos(theta);-sin(theta)])+dot([xr;yr]-[x;y],[cos(theta+pi/2);sin(theta+pi/2)]);
epsilont=dot([xr;yr]-[x;y],[cos(theta+pi/2);sin(theta+pi/2)]);
epsilon=[epsilont;epsilonn];
K=[kt 0; 0 kn];
T=[-sin(theta) -cos(theta); cos(theta) -sin(theta)];
U=T*K*epsilon;