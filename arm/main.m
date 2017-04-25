%% Main function for arm simulation

Ts=0.01;

model.m1=1;
model.m2=0.3;
model.l=1;
model.xc=0;
model.yc=1.6;
model.r=0.5;
model.spK=1000;
model.spPos=[model.xc;model.yc];
model.addRad=0.1;

l=model.l;
m1=model.m1;
m2=model.m2;
xc=model.xc;
yc=model.yc;


[~,Xr]=getCircle(model, Ts, model.r+model.addRad);
[theta1, theta2]=getIKSoln(Xr(1,1),Xr(1,2),model);
[t,Xr]=getCircle(model, Ts, model.r+model.addRad);


state0=[theta1;theta2;0;0].';
g=0;
Y=[state0];
for i=1:6000
    Tau=getTau(state0,Xr(i,:),model);
    [Tt,Yt]=ode45(@(t,y)armDynamics(t,y,model,Tau), [0, Ts], state0);
    Y=[Y;Yt(end,:)];
    state0=Yt(end,:);
end

for i=1:6000
    tau=[0;0];
    X=[model.l*cos(Y(i,1));model.l*sin(Y(i,1))]+[model.l*cos(Y(i,1)+Y(i,2)); model.l*sin(Y(i,1)+Y(i,2))];
    y=Y(i,:);
    F(i)=getForce(t,y,model,tau);
    Xact(i)=X(1);
    Yact(i)=X(2);
end

theta1=Y(end,1);
theta2=Y(end,2);
X = [l*cos(theta1) + l*cos(theta1+theta2); l*sin(theta1)+ l*sin(theta1+theta2)];

% animate(Y,model,Xr);
figure();
plot(F);
figure();
plot(Xact);
hold on;
plot(Xr(:,1));
figure();
plot(Yact);
hold on;
plot(Xr(:,2));