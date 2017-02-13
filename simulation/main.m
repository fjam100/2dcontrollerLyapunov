%% Main file  for running dynamic simulation
% Control law is provided in a separate function and is called within the
% ode45 function

model.mx=1; %Mass actuated in X dorection
model.my=0.3; % Mass actuated in Y direction
model.r=0.01; %cm to m %radius of circular part being deburred
model.spPos=[0;0]; %spindle position
model.spRad=0.05; %cm to m;
model.spK=100000;
model.cx=1;
model.cy=1;

%% Circle stuff
[timeSamples,Xr,xc,yc,Re]=getCircle();
model.spPos=[xc;yc];
model.r=norm([Xr(1,1);Xr(1,2)]-model.spPos)-model.spRad;
state0=[Xr(1,1);Xr(1,2);Xr(1,3);Xr(1,4)]; %x,y xd,yd
% figure();
% plot(Xr(:,1),Xr(:,2),'o');
[T,Y]=ode45(@(t,y)tableDynamics(t,y,timeSamples,Xr,model),[0:0.1:timeSamples(end)],state0);
animateTable(Y,model);
F=getForce(Y,model);
figure();
plot(T,Y(:,1));
hold on;
plot(timeSamples,Xr(:,1));
figure();
plot(T,Y(:,2));
hold on;
plot(timeSamples,Xr(:,2));
figure()
plot(F);
