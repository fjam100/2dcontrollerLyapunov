%% Main file  for running dynamic simulation
% Control law is provided in a separate function and is called within the
% ode45 function
%%%%%%%%%Ensure get circle has the correct radius%%%%%%%%%%%%%%%%%%%%

model.mx=1; %Mass actuated in X dorection
model.my=0.3; % Mass actuated in Y direction
model.r=0.01; %cm to m %radius of circular part being deburred
model.spPos=[0;0]; %spindle position
model.spRad=0.05; %cm to m;
model.spK=1000;
model.cx=1;
model.cy=1;

%% Circle stuff
[timeSamples,Xr,xc,yc,Re]=getCircle(model);
model.spPos=[xc;yc];
% model.r=norm([Xr(1,1);Xr(1,2)]-model.spPos)-model.spRad;
% state0=[Xr(1,1);Xr(1,2);Xr(1,3);Xr(1,4)]; %x,y xd,yd
model.r=Re-model.spRad;
state0=[model.spPos(1)+model.r+model.spRad, model.spPos(2),0,0];
T=[];
Y=[];
%% Dynamic simulation
for i=1:length(timeSamples)-1
    U=getU2(state0,Xr(i+1,:),model);
    Ucollated(i,:)=U.';
    [Tt,Yt]=ode45(@(t,y)tableDynamics(t,y,timeSamples,Xr,model,U),[timeSamples(i),timeSamples(i+1)],state0);
    state0=Yt(end,:);
    T=[T;Tt(end,:)];
    Y=[Y;Yt(end,:)];
end
%% Animation
% animateTable(Y,model);

for i=1:length(Xr(:,1))
    theta=atan2(Xr(i,2)-yc,Xr(i,1)-xc);
    Xr2(i,1)=xc+model.spRad*cos(theta);
    Xr2(i,2)=yc+model.spRad*sin(theta);
end

%% Get plots
F=getForce(Y,model);
figure();
plot(T,Y(:,1));
hold on;
plot(timeSamples,Xr2(:,1));
% figure();
hold on;
plot(T,Y(:,2));
hold on;
plot(timeSamples,Xr2(:,2));
figure()
plot(F);

plot(Y(:,1),Y(:,2),'o')
hold on;
plot(Xr2(:,1),Xr2(:,2));
%% Get input

xc=model.spPos(1);
yc=model.spPos(2);
r0=model.spRad;
for i=1:length(T)
    x=Y(i,1);
    y=Y(i,2);
    xd=Y(i,3);
    yd=Y(i,4);
    xr=Xr(i,1);
    yr=Xr(i,2);
    xrd=Xr(i,3);
    yrd=Xr(i,4);
    xrdd=Xr(i,5);
    yrdd=Xr(i,6);
    epsilon(i,:)=[((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1/2).*(xr.*((-1).*y+yc)+xc.*(y+( ...
  -1).*yr)+x.*((-1).*yc+yr)),(-1).*r0+((x+(-1).*xc).^2+(y+(-1).*yc).^2).^( ...
  1/2)];

end
