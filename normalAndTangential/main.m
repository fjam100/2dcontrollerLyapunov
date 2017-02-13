[timeSamples,Xr,xc,yc,Re]=getCircle();
model.spPos=[xc;yc];
r0=Re-35;
plot(Xr(:,1),Xr(:,2),'o')
axis('equal')