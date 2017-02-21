function animateTable(y,model)
[m,n]=size(y);
cSpindle=circleMass(model.spPos.',model.spRad);
fig=figure();
ycirc=[-183.729;-94.660;-81.150;-160.970;-304.867;-316.914]/1000;
xcirc=[-160.223;-222.451;-284.407;-389.623;-327.640;-277.457]/1000;
figure();
% plot(x,y,'o'), title(' measured points');
[xc,yc,Re,a] = circfit(xcirc,ycirc);
th = linspace(0,2*pi,20)';
xe = Re*cos(th)+xc; ye = Re*sin(th)+yc;
plot(xcirc,ycirc,'o',[xe;xe(1)],[ye;ye(1)],'-.'),
axs=axes('Parent',fig);
axis(axs, [-180, -150, -200, -180]/1000);
axis('equal');
cSpindle.plotCircle(fig,axs);
cMass=circleMass(y(1,1:2),model.r);
cMass.plotCircle(fig,axs);
for i=1:100:m
    cMass.updateState(y(i,1:2));
    cSpindle.updateState(cSpindle.centre);
    pause(0.1);
end