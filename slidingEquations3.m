%% Code from scratch to check for errors

syms mx my cx cy dx dy xd yd x y xr yr xrd yrd xrdd yrdd xc yc;
syms xc yc r0 %parameters of 'normal' circle
syms ux uy;

xdd=(ux-cx*xd)/mx;
ydd=(uy-cy*yd)/my;
% thetad=dot([xd;yd],[cos(theta+pi/2);sin(theta+pi/2)])/norm([x;y]-[xc;yc]);
% thetadd=dot([xdd;ydd],[cos(theta+pi/2);sin(theta+pi/2)])/norm([x;y]-[xc;yc]);
theta=atan2(yr-yc,xr-xc);


epsilon=[-cos(theta) -sin(theta); -sin(theta) cos(theta)]*[xr-x;yr-y];
% epsilont =  dot([xr;yr]-[x;y],[cos(theta+pi/2);sin(theta+pi/2)]);
% epsilon=[dot([xr-x; yr-y],[-cos(theta);-sin(theta)]);dot([xr-x;yr-y],[-sin(theta);cos(theta)])];
epsilont=epsilon(1);
epsilontd = diff(epsilont,xr)*xrd+diff(epsilont,yr)*yrd+ ...
            diff(epsilont,x)*xd+diff(epsilont,y)*yd;
epsilontdd =diff(epsilontd,xr)*xrd+diff(epsilontd,yr)*yrd+ ...
            diff(epsilontd,xrd)*xrdd+diff(epsilontd,yrd)*yrdd+ ...
            diff(epsilontd,x)*xd+diff(epsilontd,y)*yd+ ...
            diff(epsilontd,xd)*xdd+diff(epsilontd,yd)*ydd;
epsilonn=epsilon(2);
% epsilonn =  dot([xc+r0*cos(theta);yc+r0*sin(theta)]-[x;y],[-cos(theta); -sin(theta)])+...
%             dot([xr;yr]-[x;y],[-cos(theta); -sin(theta)]);            

epsilonnd = diff(epsilonn,xr)*xrd+diff(epsilonn,yr)*yrd+ ...
            diff(epsilonn,xrd)*xrdd+diff(epsilonn,yrd)*yrdd+ ...
            diff(epsilonn,x)*xd+diff(epsilonn,y)*yd+ ...
            diff(epsilonn,xd)*xdd+diff(epsilonn,yd)*ydd;
        
epsilonndd =diff(epsilonnd,xr)*xrd+diff(epsilonnd,yr)*yrd+ ...
            diff(epsilonnd,xrd)*xrdd+diff(epsilonnd,yrd)*yrdd+ ...
            diff(epsilonnd,x)*xd+diff(epsilonnd,y)*yd+ ...
            diff(epsilonnd,xd)*xdd+diff(epsilonnd,yd)*ydd;

% epsilon=[epsilont;epsilonn];
epsilond=[epsilontd;epsilonnd];
epsilondd=[epsilontdd;epsilonndd];

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

syms Kpxx Kpxy Kpyx Kpyy;
syms Kdxx Kdxy Kdyx Kdyy;
syms Ksxx Ksxy Ksyx Ksyy;

Kp=[Kpxx Kpxy; Kpyx Kpyy];
Kd=[Kdxx Kdxy; Kdyx Kdyy];
Ks=[Ksxx Ksxy; Ksyx Ksyy];
M=[mx 0; 0 my];

U=inv(B)*(inv(Kd)*(-inv(M)*Ks*(Kp*epsilon+Kd*epsilond)-Kp*epsilond)-subtr);
% simplify(U)
% fid = fopen('file3.m','w');
% fprintf(fid,'%s','function [res]=getU1(mx my cx cy dx dy xd yd x y xr yr xrd yrd xrdd yrdd xc yc)');
% fprintf(fid,'%s','U(1)=');
% fprintf(fid,'%s\n',expand(U(1)));
% fprintf(fid,'%s\n',);

% fid2=fopen('file2.txt','w');
% fprintf(fid2,'%s\n',expand(U(2)));
% fclose(fid);
% fclose(fid2);