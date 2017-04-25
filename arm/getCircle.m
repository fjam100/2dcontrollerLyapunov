function [t,Xr]=getCircle(model, Ts, rad)

center=[model.xc;model.yc];
thetad=0.05;
[circXt, circYt]=genCircle(center,rad,Ts,thetad);
t=(0:1:length(circXt)-1)*Ts;

for i=1:length(circXt)-1
    circXtd(i,1)=(circXt(i+1)-circXt(i))/(t(i+1)-t(i));
    circYtd(i,1)=(circYt(i+1)-circYt(i))/(t(i+1)-t(i));
end
circXtd(i+1,1)=circXtd(i);
circYtd(i+1,1)=circYtd(i);

for i=1:length(circXtd)-1
    circXtdd(i,1)=(circXtd(i+1)-circXtd(i))/(t(i+1)-t(i));
    circYtdd(i,1)=(circYtd(i+1)-circYtd(i))/(t(i+1)-t(i));
end
circXtdd(i+1)=circXtd(i);
circYtdd(i+1)=circYtd(i);
Xr=flipud([circXt.',circYt.',circXtd,circYtd,circXtdd,circYtdd]);