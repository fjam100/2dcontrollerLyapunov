syms cx mx cy my theta kt kn epsilont epsilonn;
syms dx dy xr xrd xrdd x xd xdd yr yrd yrdd y yd ydd ux uy;
syms theta thetad thetadd;
syms xt yt; %tangent unit vectors
syms r; %radius vector
% syms Kp Kd Ki;
T=[-sin(theta) -cos(theta); cos(theta) -sin(theta)];
Ktn=[kt 0;0 kn];
%epsilon=[epsilont;epsilonn];
U=[ux;uy];
e=[xr;yr]-[x;y];
ed=[xrd;yrd]-[xd;yd];
edd=[xrdd;yrdd]-([1/mx*(cx*xd+dx);1/my*(cy*yd+dy)]+U);
thetadd=1/r*[xt yt]*([-1/mx*(cx*xd+dx);-1/my*(cy*yd+dy)]+U);
Tinv=inv(T);
epsilon=Tinv*e;
Tinvd=diff(Tinv,theta)*thetad;
epsilond=Tinvd*e+Tinv*ed;
epsilondd=(diff(Tinvd,theta)*thetad+diff(Tinvd,thetad)*thetadd)*e+2*Tinvd*ed+Tinv*edd;


Ks=[0.1 0;0 0.1];
M=[mx 0;0 my];
Kp=[1 0;0 1]; Kd=[1 0; 0 1];
[coefficients,~]=coeffs(epsilondd(1,:),ux);
rem=epsilondd(1,:)-coefficients(1)*ux;
Bxx=coefficients(1);
[coefficients,~]=coeffs(epsilondd(1,:),uy);
rem=rem-coefficients(1)*uy;
Bxy=coefficients(1);
subtr(1,1)=rem;

[coefficients,~]=coeffs(epsilondd(2,:),ux);
rem=epsilondd(2,:)-coefficients(1)*ux;
Byx=coefficients(1);
[coefficients,~]=coeffs(epsilondd(2,:),uy);
rem=rem-coefficients(1)*uy;
Byy=coefficients(1);
B=[Bxx Bxy;Byx Byy];
subtr(2,1)=rem;
U=inv(Kd)*(-inv(M)*Ks*(Kp*epsilon+Kd*epsilond)+Kp*epsilond)-subtr;
simplify(U)
%% Put this U in state space form in the controller equation used by the dynamics in subfolder simulation


% S=(Kp*epsilon+)
% 
%  -(dy*mx*cos(theta) - dx*my*sin(theta) + cy*mx*yd*cos(theta) - mx*my*uy*cos(theta) - 2*mx*my*xd*cos(theta) + 2*mx*my*xrd*cos(theta) - mx*my*y*cos(theta) + mx*my*yr*cos(theta) - mx*my*yrdd*cos(theta) - cx*my*xd*sin(theta) + mx*my*ux*sin(theta) + mx*my*x*sin(theta) - mx*my*xr*sin(theta) + mx*my*xrdd*sin(theta) - 2*mx*my*yd*sin(theta) + 2*mx*my*yrd*sin(theta))/(mx*my)
%   (dx*my*cos(theta) + dy*mx*sin(theta) + cx*my*xd*cos(theta) - mx*my*ux*cos(theta) - mx*my*x*cos(theta) + mx*my*xr*cos(theta) - mx*my*xrdd*cos(theta) + 2*mx*my*yd*cos(theta) - 2*mx*my*yrd*cos(theta) + cy*mx*yd*sin(theta) - mx*my*uy*sin(theta) - 2*mx*my*xd*sin(theta) + 2*mx*my*xrd*sin(theta) - mx*my*y*sin(theta) + mx*my*yr*sin(theta) - mx*my*yrdd*sin(theta))/(mx*my)
%  

% epsilondd=
%   -(dy*mx*cos(theta) - dx*my*sin(theta) + cy*mx*yd*cos(theta) - mx*my*uy*cos(theta) - 2*mx*my*xd*cos(theta) + 2*mx*my*xrd*cos(theta) - mx*my*y*cos(theta) + mx*my*yr*cos(theta) - mx*my*yrdd*cos(theta) - cx*my*xd*sin(theta) + mx*my*ux*sin(theta) + mx*my*x*sin(theta) - mx*my*xr*sin(theta) + mx*my*xrdd*sin(theta) - 2*mx*my*yd*sin(theta) + 2*mx*my*yrd*sin(theta))/(mx*my)
%   (dx*my*cos(theta) + dy*mx*sin(theta) + cx*my*xd*cos(theta) - mx*my*ux*cos(theta) - mx*my*x*cos(theta) + mx*my*xr*cos(theta) - mx*my*xrdd*cos(theta) + 2*mx*my*yd*cos(theta) - 2*mx*my*yrd*cos(theta) + cy*mx*yd*sin(theta) - mx*my*uy*sin(theta) - 2*mx*my*xd*sin(theta) + 2*mx*my*xrd*sin(theta) - mx*my*y*sin(theta) + mx*my*yr*sin(theta) - mx*my*yrdd*sin(theta))/(mx*my)
%  epsilond=
%  
%  x*cos(theta) - xr*cos(theta) - yd*cos(theta) + yrd*cos(theta) + xd*sin(theta) - xrd*sin(theta) + y*sin(theta) - yr*sin(theta)
%  xd*cos(theta) - xrd*cos(theta) + y*cos(theta) - yr*cos(theta) - x*sin(theta) + xr*sin(theta) + yd*sin(theta) - yrd*sin(theta)
%  