function [M, coeffmat, Kd, Ks, res, Keff]=getSSdot(X,Xr,model)
%% Function to give input based on:
% theta = ArcTan[x - xc, y - yc];
% epsilontgt = 
%   Dot[{xr - x, yr - y}, {Cos[theta + Pi/2], Sin[theta + Pi/2]}];
% epsilonnorm = 
%   Dot[{xc + r0*Cos[theta] - x,  yc + r0*Sin[theta] - y}, {-Cos[theta], -Sin[theta]}];

x=X(1);
y=X(2);
xd=X(3);
yd=X(4);
xr=Xr(1);
yr=Xr(2);
xrd=Xr(3);
yrd=Xr(4);
xrdd=Xr(5);
yrdd=Xr(6);

dx=0;
dy=0;

xc=model.spPos(1);
yc=model.spPos(2);
r0=model.spRad;

theta=atan2(-model.spPos(2)+y,-model.spPos(1)+x);


epsilonn =  dot(-[xc+r0*cos(theta);yc+r0*sin(theta)]+[x;y],[-cos(theta); -sin(theta)])+dot(-[xr;yr]+[x;y],[-cos(theta); -sin(theta)]);%+...
%             dot([xc+r0*cos(theta);yc+r0*sin(theta)]-[x;y],[-cos(theta); -sin(theta)]);
epsilont =  dot(-[xr;yr]+[x;y],[cos(theta+pi/2);sin(theta+pi/2)]);
% [xr;yr]
% [x;y]
T=[-sin(theta) -cos(theta); cos(theta) -sin(theta)];
kt=1000;
kn=10;


cx=model.cx;
cy=model.cy;
mx=1;%model.mx;
my=1;%model.my;

M=[model.mx 0; 0 model.my];
Kp=[5 0; 0 5];
Kd=[0.5 0; 0 0.5];
Ks=[10 0;0 2];


theta=atan2(-model.spPos(2)+y,-model.spPos(1)+x);
thetad=((yc-y)*xd+(x-xc)*yd)/((x-xc)^2+(y-yc)^2);

% 
% epsilonn =  dot(-[xc+r0*cos(theta);yc+r0*sin(theta)]+[x;y],[-cos(theta); -sin(theta)])+dot(-[xr;yr]+[x;y],[-cos(theta); -sin(theta)]);%+...
% %             dot([xc+r0*cos(theta);yc+r0*sin(theta)]-[x;y],[-cos(theta); -sin(theta)]);
% epsilont =  dot(-[xr;yr]+[x;y],[cos(theta+pi/2);sin(theta+pi/2)]);
% % [xr;yr]
% % [x;y]
% T=[-sin(theta) -cos(theta); cos(theta) -sin(theta)];
% kt=1000;
% kn=10;

cx=model.cx;
cy=model.cy;
mx=model.mx;
my=model.my;

M=[mx 0; 0 my];
C=[cx,0;0,cy];
Kp=[5 0; 0 5];
Kd=[0.5 0; 0 0.5];
Ks=[100 0;0 20];

T=[(-1).*sin(theta),cos(theta);(-1).*cos(theta),(-1).*sin(theta)];
Td=[(-1).*thetad.*cos(theta),(-1).*thetad.*sin(theta);thetad.*sin(theta),( ...
  -1).*thetad.*cos(theta)];


epsilon=[((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-1/2).*(xr.*((-1).*y+yc)+xc.*(y+( ...
  -1).*yr)+x.*((-1).*yc+yr)),((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-1/2).* ...
  (xc.*xr+(-1).*xr.^2+x.*((-1).*xc+xr)+(-1).*y.*yc+y.*yr+yc.*yr+(-1).* ...
  yr.^2)].';
epsilond=[((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-3/2).*(xrd.*((xc+(-1).*xr).^2+( ...
  yc+(-1).*yr).^2).*((-1).*y+yr)+(-1).*((-1).*xd+xrd).*((xc+(-1).*xr).^2+( ...
  yc+(-1).*yr).^2).*((-1).*yc+yr)+(-1).*((-1).*x+xr).*((xc+(-1).*xr).^2+( ...
  yc+(-1).*yr).^2).*yrd+((-1).*xc+xr).*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2) ...
  .*((-1).*yd+yrd)+(xc+(-1).*xr).*(y+(-1).*yr).*(xc.*xrd+(-1).*xr.*xrd+( ...
  yc+(-1).*yr).*yrd)+(x+(-1).*xr).*(yc+(-1).*yr).*((-1).*xc.*xrd+xr.*xrd+( ...
  (-1).*yc+yr).*yrd)),(1/2).*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-3/2).* ...
  ((-2).*((-1).*x+xr).*xrd.*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2)+(-2).*(( ...
  -1).*xc+xr).*((-1).*xd+xrd).*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2)+(-2).*( ...
  (xc+(-1).*xr).^2+(yc+(-1).*yr).^2).*((-1).*y+yr).*yrd+(-2).*((xc+(-1).* ...
  xr).^2+(yc+(-1).*yr).^2).*((-1).*yc+yr).*((-1).*yd+yrd)+(-2).*(x+(-1).* ...
  xr).*((-1).*xc+xr).*((-1).*xc.*xrd+xr.*xrd+((-1).*yc+yr).*yrd)+(-2).*(y+ ...
  (-1).*yr).*((-1).*yc+yr).*((-1).*xc.*xrd+xr.*xrd+((-1).*yc+yr).*yrd))].'; ...
  
coeffmat=[mx.^(-1).*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-1/2).*((-1).*yc+yr), ...
  my.^(-1).*(xc+(-1).*xr).*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-1/2); ...
  mx.^(-1).*((-1).*xc+xr).*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-1/2), ...
  my.^(-1).*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-1/2).*((-1).*yc+yr)];
rem=[mx.^(-1).*my.^(-1).*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-5/2).*(cx.* ...
  my.*xd.*(xc.^2+(-2).*xc.*xr+xr.^2+(yc+(-1).*yr).^2).^2.*(yc+(-1).*yr)+ ...
  mx.*((-1).*cy.*(xc+(-1).*xr).*yd.*(xc.^2+(-2).*xc.*xr+xr.^2+(yc+(-1).* ...
  yr).^2).^2+my.*(2.*xd.*xr.^3.*xrd.*yc+(-2).*x.*xr.^2.*xrd.^2.*yc+x.* ...
  xr.^3.*xrdd.*yc+3.*xr.*xrd.^2.*y.*yc.^2+(-1).*xr.^2.*xrdd.*y.*yc.^2+2.* ...
  xd.*xr.*xrd.*yc.^3+x.*xrd.^2.*yc.^3+(-3).*xr.*xrd.^2.*yc.^3+x.*xr.* ...
  xrdd.*yc.^3+xr.^2.*xrdd.*yc.^3+(-1).*xrdd.*y.*yc.^4+xrdd.*yc.^5+(-2).* ...
  xr.^2.*xrd.*yc.^2.*yd+(-2).*xrd.*yc.^4.*yd+(-2).*xd.*xr.^3.*xrd.*yr+2.* ...
  x.*xr.^2.*xrd.^2.*yr+(-1).*x.*xr.^3.*xrdd.*yr+(-6).*xr.*xrd.^2.*y.*yc.* ...
  yr+2.*xr.^2.*xrdd.*y.*yc.*yr+(-6).*xd.*xr.*xrd.*yc.^2.*yr+(-3).*x.* ...
  xrd.^2.*yc.^2.*yr+6.*xr.*xrd.^2.*yc.^2.*yr+(-3).*x.*xr.*xrdd.*yc.^2.*yr+ ...
  (-2).*xr.^2.*xrdd.*yc.^2.*yr+4.*xrdd.*y.*yc.^3.*yr+(-4).*xrdd.*yc.^4.* ...
  yr+4.*xr.^2.*xrd.*yc.*yd.*yr+8.*xrd.*yc.^3.*yd.*yr+3.*xr.*xrd.^2.*y.* ...
  yr.^2+(-1).*xr.^2.*xrdd.*y.*yr.^2+6.*xd.*xr.*xrd.*yc.*yr.^2+3.*x.* ...
  xrd.^2.*yc.*yr.^2+(-3).*xr.*xrd.^2.*yc.*yr.^2+3.*x.*xr.*xrdd.*yc.*yr.^2+ ...
  xr.^2.*xrdd.*yc.*yr.^2+(-6).*xrdd.*y.*yc.^2.*yr.^2+6.*xrdd.*yc.^3.* ...
  yr.^2+(-2).*xr.^2.*xrd.*yd.*yr.^2+(-12).*xrd.*yc.^2.*yd.*yr.^2+(-2).* ...
  xd.*xr.*xrd.*yr.^3+(-1).*x.*xrd.^2.*yr.^3+(-1).*x.*xr.*xrdd.*yr.^3+4.* ...
  xrdd.*y.*yc.*yr.^3+(-4).*xrdd.*yc.^2.*yr.^3+8.*xrd.*yc.*yd.*yr.^3+(-1).* ...
  xrdd.*y.*yr.^4+xrdd.*yc.*yr.^4+(-2).*xrd.*yd.*yr.^4+2.*xd.*xr.^4.*yrd+( ...
  -2).*x.*xr.^3.*xrd.*yrd+4.*xr.^2.*xrd.*y.*yc.*yrd+2.*xd.*xr.^2.*yc.^2.* ...
  yrd+4.*x.*xr.*xrd.*yc.^2.*yrd+(-4).*xr.^2.*xrd.*yc.^2.*yrd+(-2).*xrd.* ...
  y.*yc.^3.*yrd+2.*xrd.*yc.^4.*yrd+(-2).*xr.^3.*yc.*yd.*yrd+(-2).*xr.* ...
  yc.^3.*yd.*yrd+(-4).*xr.^2.*xrd.*y.*yr.*yrd+(-4).*xd.*xr.^2.*yc.*yr.* ...
  yrd+(-8).*x.*xr.*xrd.*yc.*yr.*yrd+4.*xr.^2.*xrd.*yc.*yr.*yrd+6.*xrd.*y.* ...
  yc.^2.*yr.*yrd+(-6).*xrd.*yc.^3.*yr.*yrd+2.*xr.^3.*yd.*yr.*yrd+6.*xr.* ...
  yc.^2.*yd.*yr.*yrd+2.*xd.*xr.^2.*yr.^2.*yrd+4.*x.*xr.*xrd.*yr.^2.*yrd+( ...
  -6).*xrd.*y.*yc.*yr.^2.*yrd+6.*xrd.*yc.^2.*yr.^2.*yrd+(-6).*xr.*yc.*yd.* ...
  yr.^2.*yrd+2.*xrd.*y.*yr.^3.*yrd+(-2).*xrd.*yc.*yr.^3.*yrd+2.*xr.*yd.* ...
  yr.^3.*yrd+xr.^3.*y.*yrd.^2+3.*x.*xr.^2.*yc.*yrd.^2+(-1).*xr.^3.*yc.* ...
  yrd.^2+(-2).*xr.*y.*yc.^2.*yrd.^2+2.*xr.*yc.^3.*yrd.^2+(-3).*x.*xr.^2.* ...
  yr.*yrd.^2+4.*xr.*y.*yc.*yr.*yrd.^2+(-4).*xr.*yc.^2.*yr.*yrd.^2+(-2).* ...
  xr.*y.*yr.^2.*yrd.^2+2.*xr.*yc.*yr.^2.*yrd.^2+(-1).*xc.^5.*yrdd+x.* ...
  xr.^4.*yrdd+(-1).*xr.^3.*y.*yc.*yrdd+x.*xr.^2.*yc.^2.*yrdd+xr.^3.* ...
  yc.^2.*yrdd+(-1).*xr.*y.*yc.^3.*yrdd+xr.*yc.^4.*yrdd+xr.^3.*y.*yr.*yrdd+ ...
  (-2).*x.*xr.^2.*yc.*yr.*yrdd+(-1).*xr.^3.*yc.*yr.*yrdd+3.*xr.*y.*yc.^2.* ...
  yr.*yrdd+(-3).*xr.*yc.^3.*yr.*yrdd+x.*xr.^2.*yr.^2.*yrdd+(-3).*xr.*y.* ...
  yc.*yr.^2.*yrdd+3.*xr.*yc.^2.*yr.^2.*yrdd+xr.*y.*yr.^3.*yrdd+(-1).*xr.* ...
  yc.*yr.^3.*yrdd+xc.^4.*(xrdd.*(yc+(-1).*yr)+2.*xd.*yrd+(-2).*xrd.*yrd+ ...
  x.*yrdd+4.*xr.*yrdd)+(-1).*xc.^3.*(x.*xrdd.*yc+3.*xr.*xrdd.*yc+(-2).* ...
  xrd.^2.*(yc+(-1).*yr)+(-1).*x.*xrdd.*yr+(-3).*xr.*xrdd.*yr+(-2).*(x+3.* ...
  xr).*xrd.*yrd+(-2).*yc.*yd.*yrd+2.*yd.*yr.*yrd+y.*yrd.^2+2.*yc.*yrd.^2+( ...
  -3).*yr.*yrd.^2+2.*xd.*(xrd.*(yc+(-1).*yr)+4.*xr.*yrd)+4.*x.*xr.*yrdd+ ...
  6.*xr.^2.*yrdd+(-1).*y.*yc.*yrdd+2.*yc.^2.*yrdd+y.*yr.*yrdd+(-3).*yc.* ...
  yr.*yrdd+yr.^2.*yrdd)+xc.^2.*((-4).*xr.*xrd.^2.*yc+3.*xr.^2.*xrdd.*yc+( ...
  -1).*xrdd.*y.*yc.^2+2.*xrdd.*yc.^3+(-2).*xrd.*yc.^2.*yd+4.*xr.*xrd.^2.* ...
  yr+(-3).*xr.^2.*xrdd.*yr+2.*xrdd.*y.*yc.*yr+(-5).*xrdd.*yc.^2.*yr+4.* ...
  xrd.*yc.*yd.*yr+(-1).*xrdd.*y.*yr.^2+4.*xrdd.*yc.*yr.^2+(-2).*xrd.*yd.* ...
  yr.^2+(-1).*xrdd.*yr.^3+(-6).*xr.^2.*xrd.*yrd+4.*xrd.*y.*yc.*yrd+(-6).* ...
  xr.*yc.*yd.*yrd+(-4).*xrd.*y.*yr.*yrd+(-4).*xrd.*yc.*yr.*yrd+6.*xr.*yd.* ...
  yr.*yrd+4.*xrd.*yr.^2.*yrd+3.*xr.*y.*yrd.^2+3.*xr.*yc.*yrd.^2+(-6).*xr.* ...
  yr.*yrd.^2+2.*xd.*(3.*xr.*xrd.*(yc+(-1).*yr)+6.*xr.^2.*yrd+(yc+(-1).*yr) ...
  .^2.*yrd)+4.*xr.^3.*yrdd+(-3).*xr.*y.*yc.*yrdd+5.*xr.*yc.^2.*yrdd+3.* ...
  xr.*y.*yr.*yrdd+(-7).*xr.*yc.*yr.*yrdd+2.*xr.*yr.^2.*yrdd+x.*((-2).* ...
  xrd.^2.*(yc+(-1).*yr)+3.*xr.*xrdd.*(yc+(-1).*yr)+(-6).*xr.*xrd.*yrd+6.* ...
  xr.^2.*yrdd+(yc+(-1).*yr).*(3.*yrd.^2+yc.*yrdd+(-1).*yr.*yrdd)))+(-1).* ...
  xc.*((-2).*xr.^2.*xrd.^2.*yc+xr.^3.*xrdd.*yc+3.*xrd.^2.*y.*yc.^2+(-2).* ...
  xr.*xrdd.*y.*yc.^2+(-2).*xrd.^2.*yc.^3+3.*xr.*xrdd.*yc.^3+(-4).*xr.* ...
  xrd.*yc.^2.*yd+2.*xr.^2.*xrd.^2.*yr+(-1).*xr.^3.*xrdd.*yr+(-6).*xrd.^2.* ...
  y.*yc.*yr+4.*xr.*xrdd.*y.*yc.*yr+3.*xrd.^2.*yc.^2.*yr+(-7).*xr.*xrdd.* ...
  yc.^2.*yr+8.*xr.*xrd.*yc.*yd.*yr+3.*xrd.^2.*y.*yr.^2+(-2).*xr.*xrdd.*y.* ...
  yr.^2+5.*xr.*xrdd.*yc.*yr.^2+(-4).*xr.*xrd.*yd.*yr.^2+(-1).*xrd.^2.* ...
  yr.^3+(-1).*xr.*xrdd.*yr.^3+(-2).*xr.^3.*xrd.*yrd+8.*xr.*xrd.*y.*yc.* ...
  yrd+(-4).*xr.*xrd.*yc.^2.*yrd+(-6).*xr.^2.*yc.*yd.*yrd+(-2).*yc.^3.*yd.* ...
  yrd+(-8).*xr.*xrd.*y.*yr.*yrd+6.*xr.^2.*yd.*yr.*yrd+6.*yc.^2.*yd.*yr.* ...
  yrd+4.*xr.*xrd.*yr.^2.*yrd+(-6).*yc.*yd.*yr.^2.*yrd+2.*yd.*yr.^3.*yrd+ ...
  3.*xr.^2.*y.*yrd.^2+(-2).*y.*yc.^2.*yrd.^2+2.*yc.^3.*yrd.^2+(-3).* ...
  xr.^2.*yr.*yrd.^2+4.*y.*yc.*yr.*yrd.^2+(-4).*yc.^2.*yr.*yrd.^2+(-2).*y.* ...
  yr.^2.*yrd.^2+2.*yc.*yr.^2.*yrd.^2+2.*xd.*(3.*xr.^2.*xrd.*(yc+(-1).*yr)+ ...
  xrd.*(yc+(-1).*yr).^3+4.*xr.^3.*yrd+2.*xr.*(yc+(-1).*yr).^2.*yrd)+ ...
  xr.^4.*yrdd+(-3).*xr.^2.*y.*yc.*yrdd+4.*xr.^2.*yc.^2.*yrdd+(-1).*y.* ...
  yc.^3.*yrdd+yc.^4.*yrdd+3.*xr.^2.*y.*yr.*yrdd+(-5).*xr.^2.*yc.*yr.*yrdd+ ...
  3.*y.*yc.^2.*yr.*yrdd+(-3).*yc.^3.*yr.*yrdd+xr.^2.*yr.^2.*yrdd+(-3).*y.* ...
  yc.*yr.^2.*yrdd+3.*yc.^2.*yr.^2.*yrdd+y.*yr.^3.*yrdd+(-1).*yc.*yr.^3.* ...
  yrdd+x.*(3.*xr.^2.*(xrdd.*yc+(-1).*xrdd.*yr+(-2).*xrd.*yrd)+(yc+(-1).* ...
  yr).^2.*(xrdd.*yc+(-1).*xrdd.*yr+4.*xrd.*yrd)+4.*xr.^3.*yrdd+2.*xr.*(yc+ ...
  (-1).*yr).*((-2).*xrd.^2+3.*yrd.^2+yc.*yrdd+(-1).*yr.*yrdd)))))),mx.^( ...
  -1).*my.^(-1).*((xc+(-1).*xr).^2+(yc+(-1).*yr).^2).^(-5/2).*(cx.*my.* ...
  xd.*(xc+(-1).*xr).*(xc.^2+(-2).*xc.*xr+xr.^2+(yc+(-1).*yr).^2).^2+mx.*( ...
  cy.*yd.*(xc.^2+(-2).*xc.*xr+xr.^2+(yc+(-1).*yr).^2).^2.*(yc+(-1).*yr)+ ...
  my.*(xc.^5.*xrdd+(-1).*xr.^5.*xrdd+xr.^4.*(2.*yd.*yrd+(-1).*yrd.^2+(y+( ...
  -1).*yr).*yrdd)+xc.^4.*((-5).*xr.*xrdd+2.*yd.*yrd+(-2).*yrd.^2+y.*yrdd+ ...
  yc.*yrdd+(-2).*yr.*yrdd)+xr.^3.*(xrdd.*(yc+(-1).*yr).*(y+(-3).*yc+2.*yr) ...
  +2.*xd.*yc.*yrd+(-2).*xd.*yr.*yrd+(-1).*x.*yrd.^2+2.*xrd.*(yc.*yd+(-1).* ...
  yd.*yr+(-1).*y.*yrd+yr.*yrd)+x.*yc.*yrdd+(-1).*x.*yr.*yrdd)+xr.^2.*(yc+( ...
  -1).*yr).*(x.*xrdd.*(yc+(-1).*yr)+xrd.^2.*((-2).*y+yc+yr)+2.*yc.*yd.* ...
  yrd+(-2).*yd.*yr.*yrd+3.*y.*yrd.^2+(-4).*yc.*yrd.^2+yr.*yrd.^2+2.*xrd.*( ...
  xd.*(yc+(-1).*yr)+(-2).*x.*yrd)+y.*yc.*yrdd+yc.^2.*yrdd+(-1).*y.*yr.* ...
  yrdd+(-3).*yc.*yr.*yrdd+2.*yr.^2.*yrdd)+(yc+(-1).*yr).^3.*(xrd.^2.*(y+( ...
  -2).*yc+yr)+2.*xrd.*(xd.*(yc+(-1).*yr)+x.*yrd)+(yc+(-1).*yr).*(x.*xrdd+( ...
  yc+(-1).*yr).*yrdd))+(-1).*xr.*(yc+(-1).*yr).^2.*((-1).*xrdd.*(yc+(-1).* ...
  yr).*(y+(-2).*yc+yr)+(-2).*(xd.*(yc+(-1).*yr).*yrd+xrd.*(yc.*yd+(-1).* ...
  yd.*yr+2.*y.*yrd+(-3).*yc.*yrd+yr.*yrd))+x.*(3.*xrd.^2+(-2).*yrd.^2+(-1) ...
  .*yc.*yrdd+yr.*yrdd))+xc.^3.*(10.*xr.^2.*xrdd+(-2).*xrd.*yc.*yd+2.*xrd.* ...
  yd.*yr+(-1).*xrdd.*(yc+(-1).*yr).*(y+(-2).*yc+yr)+2.*xrd.*y.*yrd+(-2).* ...
  xd.*yc.*yrd+4.*xrd.*yc.*yrd+2.*xd.*yr.*yrd+(-6).*xrd.*yr.*yrd+x.*yrd.^2+ ...
  (-1).*x.*yc.*yrdd+x.*yr.*yrdd+xr.*((-8).*yd.*yrd+7.*yrd.^2+(-4).*y.* ...
  yrdd+(-3).*yc.*yrdd+7.*yr.*yrdd))+xc.^2.*((-10).*xr.^3.*xrdd+3.*xr.^2.*( ...
  4.*yd.*yrd+(-3).*yrd.^2+(2.*y+yc+(-3).*yr).*yrdd)+xr.*(xrdd.*(yc+(-1).* ...
  yr).*(3.*y+(-7).*yc+4.*yr)+6.*xd.*yc.*yrd+(-6).*xd.*yr.*yrd+(-3).*x.* ...
  yrd.^2+2.*xrd.*(3.*yc.*yd+(-3).*yd.*yr+(-3).*y.*yrd+(-4).*yc.*yrd+7.* ...
  yr.*yrd)+3.*x.*yc.*yrdd+(-3).*x.*yr.*yrdd)+(-1).*(yc+(-1).*yr).*(2.* ...
  xrd.^2.*(y+yc+(-2).*yr)+x.*xrdd.*((-1).*yc+yr)+(-2).*yc.*yd.*yrd+2.*yd.* ...
  yr.*yrd+(-3).*y.*yrd.^2+2.*yc.*yrd.^2+yr.*yrd.^2+xrd.*((-2).*xd.*yc+2.* ...
  xd.*yr+4.*x.*yrd)+(-1).*y.*yc.*yrdd+(-2).*yc.^2.*yrdd+y.*yr.*yrdd+5.* ...
  yc.*yr.*yrdd+(-3).*yr.^2.*yrdd))+xc.*(5.*xr.^4.*xrdd+(-1).*xr.^3.*(8.* ...
  yd.*yrd+(-5).*yrd.^2+(4.*y+yc+(-5).*yr).*yrdd)+xr.^2.*(xrdd.*((-3).*y+ ...
  8.*yc+(-5).*yr).*(yc+(-1).*yr)+(-6).*xd.*yc.*yrd+6.*xd.*yr.*yrd+3.*x.* ...
  yrd.^2+xrd.*((-6).*yc.*yd+6.*yd.*yr+6.*y.*yrd+4.*yc.*yrd+(-10).*yr.*yrd) ...
  +(-3).*x.*yc.*yrdd+3.*x.*yr.*yrdd)+xr.*(yc+(-1).*yr).*(xrd.^2.*(4.*y+yc+ ...
  (-5).*yr)+2.*x.*xrdd.*((-1).*yc+yr)+(-4).*yc.*yd.*yrd+4.*yd.*yr.*yrd+( ...
  -6).*y.*yrd.^2+6.*yc.*yrd.^2+xrd.*((-4).*xd.*yc+4.*xd.*yr+8.*x.*yrd)+( ...
  -2).*y.*yc.*yrdd+(-3).*yc.^2.*yrdd+2.*y.*yr.*yrdd+8.*yc.*yr.*yrdd+(-5).* ...
  yr.^2.*yrdd)+(yc+(-1).*yr).^2.*((-2).*xrd.*yc.*yd+(-1).*xrdd.*(y+(-1).* ...
  yc).*(yc+(-1).*yr)+2.*xrd.*yd.*yr+(-4).*xrd.*y.*yrd+(-2).*xd.*yc.*yrd+ ...
  4.*xrd.*yc.*yrd+2.*xd.*yr.*yrd+x.*(3.*xrd.^2+(-2).*yrd.^2+(-1).*yc.* ...
  yrdd+yr.*yrdd))))))].';

% res=inv(coeffmat)*(inv(Kd)*(-inv(eye(2))*Ks*(Kp*epsilon+Kd*epsilond)-Kp*epsilond)-rem);
S=Kp*epsilon+Kd*epsilond;
delta=eye(2)-[model.mx 0; 0 model.my]*inv([mx 0; 0 my]);
res=S.'*(-Ks*S-Kd*coeffmat*delta*inv(coeffmat)*Ks*S+Kd*coeffmat*delta*inv(coeffmat)*rem);
U=inv(coeffmat)*(inv(Kd)*(-inv(eye(2))*Ks*(Kp*epsilon+Kd*epsilond)-Kp*epsilond)-rem);


Xdd=inv(M)*(U-C*[xd;yd]);
xdd=Xdd(1);
ydd=Xdd(2);
Tdd=[((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1/2).*(y+(-1).*yc).*(xd.*((x+(-1) ...
  .*xc).^2+(y+(-1).*yc).^2).^(-1).*((-1).*y+yc)+(x+(-1).*xc).*((x+(-1).* ...
  xc).^2+(y+(-1).*yc).^2).^(-1).*yd).^2+(-1).*(x+(-1).*xc).*((x+(-1).*xc) ...
  .^2+(y+(-1).*yc).^2).^(-1/2).*((-1).*xd.*((x+(-1).*xc).^2+(y+(-1).*yc) ...
  .^2).^(-2).*((-1).*y+yc).*(2.*(x+(-1).*xc).*xd+2.*(y+(-1).*yc).*yd)+(-1) ...
  .*(x+(-1).*xc).*((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-2).*yd.*(2.*(x+(-1) ...
  .*xc).*xd+2.*(y+(-1).*yc).*yd)+((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1).* ...
  ((-1).*y+yc).*xdd+(x+(-1).*xc).*((x+(-1).*xc).^2+(y+(-1).*yc).^2) ...
  .^(-1).*ydd),(-1).*(x+(-1).*xc).*((x+(-1).*xc).^2+(y+(-1).*yc).^2) ...
  .^(-1/2).*(xd.*((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1).*((-1).*y+yc)+(x+ ...
  (-1).*xc).*((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1).*yd).^2+(-1).*((x+( ...
  -1).*xc).^2+(y+(-1).*yc).^2).^(-1/2).*(y+(-1).*yc).*((-1).*xd.*((x+(-1) ...
  .*xc).^2+(y+(-1).*yc).^2).^(-2).*((-1).*y+yc).*(2.*(x+(-1).*xc).*xd+2.*( ...
  y+(-1).*yc).*yd)+(-1).*(x+(-1).*xc).*((x+(-1).*xc).^2+(y+(-1).*yc).^2) ...
  .^(-2).*yd.*(2.*(x+(-1).*xc).*xd+2.*(y+(-1).*yc).*yd)+((x+(-1).*xc).^2+( ...
  y+(-1).*yc).^2).^(-1).*((-1).*y+yc).*xdd+(x+(-1).*xc).*((x+(-1).* ...
  xc).^2+(y+(-1).*yc).^2).^(-1).*ydd);(x+(-1).*xc).*((x+(-1).*xc).^2+ ...
  (y+(-1).*yc).^2).^(-1/2).*(xd.*((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1).* ...
  ((-1).*y+yc)+(x+(-1).*xc).*((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1).*yd) ...
  .^2+((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1/2).*(y+(-1).*yc).*((-1).*xd.* ...
  ((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-2).*((-1).*y+yc).*(2.*(x+(-1).*xc) ...
  .*xd+2.*(y+(-1).*yc).*yd)+(-1).*(x+(-1).*xc).*((x+(-1).*xc).^2+(y+(-1).* ...
  yc).^2).^(-2).*yd.*(2.*(x+(-1).*xc).*xd+2.*(y+(-1).*yc).*yd)+((x+(-1).* ...
  xc).^2+(y+(-1).*yc).^2).^(-1).*((-1).*y+yc).*xdd+(x+(-1).*xc).*((x+ ...
  (-1).*xc).^2+(y+(-1).*yc).^2).^(-1).*ydd),((x+(-1).*xc).^2+(y+(-1) ...
  .*yc).^2).^(-1/2).*(y+(-1).*yc).*(xd.*((x+(-1).*xc).^2+(y+(-1).*yc).^2) ...
  .^(-1).*((-1).*y+yc)+(x+(-1).*xc).*((x+(-1).*xc).^2+(y+(-1).*yc).^2).^( ...
  -1).*yd).^2+(-1).*(x+(-1).*xc).*((x+(-1).*xc).^2+(y+(-1).*yc).^2).^( ...
  -1/2).*((-1).*xd.*((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-2).*((-1).*y+yc) ...
  .*(2.*(x+(-1).*xc).*xd+2.*(y+(-1).*yc).*yd)+(-1).*(x+(-1).*xc).*((x+(-1) ...
  .*xc).^2+(y+(-1).*yc).^2).^(-2).*yd.*(2.*(x+(-1).*xc).*xd+2.*(y+(-1).* ...
  yc).*yd)+((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1).*((-1).*y+yc).*xdd ...
  +(x+(-1).*xc).*((x+(-1).*xc).^2+(y+(-1).*yc).^2).^(-1).*ydd)];

% 
% 
% Knorm=200;
% Kpy=(Knorm-dot(M*inv(T)*(2*Td*inv(T)*Td*inv(T))*[0;1],[-cos(theta);-sin(theta)]))*Kd(2,2)/(Ks(2,2)*(mx*cos(theta)^2+my*sin(theta)^2));
% Ktgt=1000;
% Kpx=(Ktgt-dot(M*inv(T)*(2*Td*inv(T)*Td*inv(T))*[1;0],[cos(theta+pi/2);sin(theta+pi/2)]))*Kd(1,1)/(Ks(1,1)*(my*cos(theta)^2+mx*sin(theta)^2));
% Kp=[Kpx,0;0,Kpy];

Keff=dot(-M*inv(T)*(-inv(Kd)*Ks*Kp+2*Td*inv(T)*Td*inv(T))*[0;1],[-cos(theta);-sin(theta)]);
Keff2=dot(-M*inv(T)*(-inv(Kd)*Ks*Kp+2*Td*inv(T)*Td*inv(T))*[1;0],[cos(theta+pi/2);sin(theta+pi/2)]);