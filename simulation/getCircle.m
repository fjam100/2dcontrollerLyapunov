function [t,Xr, xc, yc,Re]=getCircle(model)
y=[-183.729;-94.660;-81.150;-160.970;-304.867;-316.914]/1000;
x=[-160.223;-222.451;-284.407;-389.623;-327.640;-277.457]/1000;
% figure();
% plot(x,y,'o'), title(' measured points');
[xc,yc,Re,a] = circfit(x,y);
th = linspace(0,2*pi,20)';
xe = Re*cos(th)+xc; ye = Re*sin(th)+yc;
% plot(x,y,'o',[xe;xe(1)],[ye;ye(1)],'-.'),
% title(' measured and fitted circles')
% legend('measured','fitted')
% text(xc,yc,sprintf('center (%g , %g );  R=%g',xc,yc,Re))
% xlabel x, ylabel y 
% axis equal

% Other parameters
thetad=0.25;
Ts=0.01;
zeta=0.8;
% Generate trajectory
center=[xc;yc];
rad=Re;

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
Xr=[circXt.',circYt.',circXtd,circYtd,circXtdd,circYtdd];