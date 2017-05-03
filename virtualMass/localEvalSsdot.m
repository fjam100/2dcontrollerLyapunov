model.mx=1; %Mass actuated in X dorection
model.my=0.3; % Mass actuated in Y direction
model.r=0.01; %cm to m %radius of circular part being deburred
model.spPos=[0;0]; %spindle position
model.spRad=0.05; %cm to m;
model.spK=100000;
model.cx=1;
model.cy=1;

%% Circle stuff
[timeSamples,Xr,xc,yc,Re]=getCircle(model);
model.spPos=[xc;yc];
% model.r=norm([Xr(1,1);Xr(1,2)]-model.spPos)-model.spRad;
% state0=[Xr(1,1);Xr(1,2);Xr(1,3);Xr(1,4)]; %x,y xd,yd
model.r=Re-model.spRad;
x=(-1:0.01:1);
y=(-1:0.01:1);
xd=(-10:0.01:10);
yd=(-10:0.01:10);