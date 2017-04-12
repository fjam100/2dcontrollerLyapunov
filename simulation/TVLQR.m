clear all;
load('inputs.mat');
Ts=0.001;
M=[1 0; 0 0.3];
C=[1 0; 0 1];
Q=[100000 0 0 0; 0 100000 0 0; 0 0 10000 0; 0 0 0 10000];
R=eye(2);
A=[zeros(2,2), eye(2); zeros(2,2) -M\C];
Ad=(eye(size(A))+A*Ts);
B=[zeros(2,2); inv(M)];
Bd=Ts*B;
[K,S{length(Ucollated)},~]=lqrd(A,B,Q,R,Ts);

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
TwithLQR=0;
YwithLQR=state0;
%% Dynamic simulation
for i=1:length(timeSamples)-1
    U=Ucollated(i,:).'+K*(Y(i,:).'-YwithLQR(end,:).');
    i
    ([Y(i,1:2).';0;0]-YwithLQR(end,:).')
    K*([Y(i,1:2).';0;0]-YwithLQR(end,:).')
%     [MCollated{i}, coeffmatCollated{i}, KdCollated{i},KsCollated{i}, SsdotCollated{i}, KeffCollated{i}]=getSSdot(state0,Xr(i+1,:),model);
    [Tt,Yt]=ode45(@(t,y)tableDynamics(t,y,timeSamples,Y(i,:).',model,U),[timeSamples(i),timeSamples(i+1)],state0);
    state0=Yt(end,:);
    TwithLQR=[TwithLQR;Tt(end,:)];
    YwithLQR=[YwithLQR;Yt(end,:)];
end


for i=1:length(Xr(:,1))
    theta=atan2(Xr(i,2)-yc,Xr(i,1)-xc);
    Xr2(i,1)=xc+model.spRad*cos(theta);
    Xr2(i,2)=yc+model.spRad*sin(theta);
end

%% Get plots
F=getForce(YwithLQR,model);
figure();
plot(TwithLQR,YwithLQR(:,1));
hold on;
plot(timeSamples,Y(:,1));
figure();
hold on;
plot(TwithLQR,YwithLQR(:,2));
hold on;

plot(timeSamples,Y(:,2));
figure()
plot(F);
% figure();
% plot(Ystab(:,1),Ystab(:,2),'o')
% hold on;
% plot(Xr2(:,1),Xr2(:,2));



