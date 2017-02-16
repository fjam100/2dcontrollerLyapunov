%% Main file  for running dynamic simulation
% Control law is provided in a separate function and is called within the
% ode45 function
%%%%%%%%%Ensure get circle has the correct radius%%%%%%%%%%%%%%%%%%%%

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
state0=[model.spPos(1)+model.r+model.spRad, model.spPos(2),0,0];

%% Dynamic simulation
[T,Y]=ode45(@(t,y)tableDynamics(t,y,timeSamples,Xr,model),[0:0.1:timeSamples(end)],state0);

%% Animation
animateTable(Y,model);

%% Get plots
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
