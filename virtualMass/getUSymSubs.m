function res = getU( X,Xr,model )
%get 2X1 vector input U based on 
%   Detailed explanation goes here
xact=X(1);
yact=X(2);
xdact=X(3);
ydact=X(4);
xract=Xr(1);
yract=Xr(2);
xrdact=Xr(3);
yrdact=Xr(4);
xrddact=Xr(5);
yrddact=Xr(6);

dxact=0;
dyact=0;

xcact=model.spPos(1);
ycact=model.spPos(2);
r0act=3.5/100;

thetaact=atan2(-model.spPos(2)+yact,-model.spPos(1)+xact);


epsilonn =  dot(-[xcact+r0act*cos(thetaact);ycact+r0act*sin(thetaact)]+[xact;yact],[-cos(thetaact); -sin(thetaact)])+dot([xract;yract]-[xact;yact],[-cos(thetaact); -sin(thetaact)]);%+...
%             dot([xc+r0act*cos(theta);yc+r0act*sin(theta)]-[xact;y],[-cos(theta); -sin(theta)]);
epsilont =  dot(-[xract;yract]+[xact;yact],[cos(thetaact+pi/2);sin(thetaact+pi/2)]);
% [xr;yr]
% [xact;y]
T=[-sin(thetaact) -cos(thetaact); cos(thetaact) -sin(thetaact)];
kt=1000;
kn=10;


cxact=model.cx;
cyact=model.cy;
mxact=model.mx;
myact=model.my;

Kpxxact=-1; Kpxyact=0; Kpyxact=0; Kpyyact=-1;
Kdxxact=-1; Kdxyact=0; Kdyxact=0; Kdyyact=-1;
Ksxxact=1; Ksxyact=0; Ksyxact=0; Ksyyact=1;


%% Using tangential and nomral force transformation
% U=T*[-kt 0; 0 -kn]*[epsilont;epsilonn];

%% Using saved symbolic expression
load('outTangentCircleNomralAnotherCircle.mat');
syms theta mx my cx cy dx dy xd yd x y xr yr xrd yrd xrdd yrdd xc yc r0;
syms Kpxx Kpxy Kpyx Kpyy Kdxx Kdxy Kdyx Kdyy Ksxx Ksxy Ksyx Ksyy;
allsyms=[theta mx my cx cy dx dy xd yd x y xr yr xrd yrd xrdd yrdd xc yc r0 ...
        Kpxx Kpxy Kpyx Kpyy Kdxx Kdxy Kdyx Kdyy Ksxx Ksxy Ksyx Ksyy];
actvars=[thetaact mxact myact cxact cyact dxact dyact xdact ydact xact yact xract yract xrdact yrdact xrddact yrddact xcact ycact r0act ...
        Kpxxact Kpxyact Kpyxact Kpyyact Kdxxact Kdxyact Kdyxact Kdyyact Ksxxact Ksxyact Ksyxact Ksyyact];
res=double(subs(U,allsyms,actvars));

