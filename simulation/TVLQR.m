load('allDataForTVLQR.mat');
Ts=0.001;
M=[1 0; 0 0.3];
C=[1 0; 0 1];
Q=eye(4);
R=eye(2);
A=[zeros(2,2), eye(2); zeros(2,2) -M\C];
Ad=(eye(size(A))+A*Ts);
B=[zeros(2,2); inv(M)];
Bd=Ts*B;
[K,S{length(Ucollated)},~]=lqrd(A,B,Q,R,Ts);



state0=[model.spPos(1)+model.r+model.spRad, model.spPos(2),0,0];
Tstab=[0];
Ystab=[state0];
%% Dynamic simulation
for i=1:length(timeSamples)-1
    U=Ucollated(i,:).'-K*([Xr(i,1:2).';0;0]-Ystab(end,:).');
    i
    ([Xr(i,1:2).';0;0]-Ystab(end,:).')
    K*([Xr(i,1:2).';0;0]-Ystab(end,:).')
%     [MCollated{i}, coeffmatCollated{i}, KdCollated{i},KsCollated{i}, SsdotCollated{i}, KeffCollated{i}]=getSSdot(state0,Xr(i+1,:),model);
    [Tt,Yt]=ode45(@(t,y)tableDynamics(t,y,timeSamples,Xr,model,U),[timeSamples(i),timeSamples(i+1)],state0);
    state0=Yt(end,:);
    Tstab=[Tstab;Tt(end,:)];
    Ystab=[Ystab;Yt(end,:)];
end


for i=1:length(Xr(:,1))
    theta=atan2(Xr(i,2)-yc,Xr(i,1)-xc);
    Xr2(i,1)=xc+model.spRad*cos(theta);
    Xr2(i,2)=yc+model.spRad*sin(theta);
end

%% Get plots
F=getForce(Ystab,model);
figure();
plot(Tstab,Ystab(:,1));
hold on;
plot(timeSamples,Xr(:,1));
figure();
hold on;
plot(Tstab,Ystab(:,2));
hold on;

plot(timeSamples,Xr(:,2));
figure()
plot(F);
figure();
plot(Ystab(:,1),Ystab(:,2),'o')
hold on;
plot(Xr2(:,1),Xr2(:,2));



