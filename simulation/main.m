%% Main file  for running dynamic simulation
% Control law is provided in a separate function and is called within the
% ode45 function
%%%%%%%%%Ensure get circle has the correct radius%%%%%%%%%%%%%%%%%%%%
clear all;
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
T=[0];
Y=[state0];

Ts=0.001;
M=[1 0; 0 0.3];
C=[1 0; 0 1];
kalman.A=[zeros(2,2), eye(2), zeros(2,2); zeros(2,2) -M\C, -inv(M); zeros(2,6)];
kalman.Ad=(eye(size(kalman.A))+kalman.A*Ts);
kalman.B=[zeros(2,2); inv(M); zeros(2,2)];
kalman.H=[eye(4), zeros(4,2)];
kalman.Bd=Ts*kalman.B;
kalman.P=zeros(6,6);
kalman.Q=eye(6)*0.01;
kalman.R=eye(4)*0.000001;



kalman.X=[state0.'; 0; 0];
%% Dynamic simulation
for i=1:length(timeSamples)-1
    [U, Knormal(i), epsilonn(i), epsilont(i)]=getU2(state0,Xr(i+1,:),model);
    theta=atan2(-model.spPos(2)+Y(end,2),-model.spPos(1)+Y(end,1));
    % Kalman model update
    kalman.X=kalman.Ad*kalman.X+kalman.Bd*(U);
    kalman.P=kalman.Ad*kalman.P*kalman.Ad.'+kalman.Q;
    
    Ucollated(i,:)=U.';
    [Tt,Yt]=ode45(@(t,y)tableDynamics(t,y,timeSamples,Xr,model,U),[timeSamples(i),timeSamples(i+1)],state0);
    state0=Yt(end,:);
    T=[T;Tt(end,:)];
    Y=[Y;Yt(end,:)];
    
    % Kalman measurement update
    kalman.K=kalman.P*kalman.H.'/(kalman.H*kalman.P*kalman.H.'+kalman.R);
    kalman.X=kalman.X+kalman.K*(Yt(end,:).'-kalman.H*kalman.X);
    kalman.P=(eye(6)-kalman.K*kalman.H)*kalman.P;

%     F=norm(model.spK*((model.r+model.spRad)-norm(model.spPos-[state0(1);state0(2)])))*[-cos(theta);-sin(theta)]
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
plot(timeSamples,Xr(:,1));
figure();
hold on;
plot(T,Y(:,2));
hold on;

plot(timeSamples,Xr(:,2));
figure()
plot(F);
figure();
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
% 
% 
% for i=1:length(MCollated)
%    temp=MCollated{i}*inv(coeffmatCollated{i})*inv(KdCollated{i})*KsCollated{i};
%    Keffective(i)=temp(2);
% end
% 
% 
% for i=1:length(MCollated)
%    temp=MCollated{i}*inv(coeffmatCollated{i})*inv(KdCollated{i})*KsCollated{i}*[0;1];
%    Keffective(i)=temp(2);
% end

