function [theta1, theta2]=getIKSoln(x,y,model)
l=model.l;
theta2=atan2(sqrt(1-((x^2+y^2-l^2-l^2)/(2*l*l))^2), (x^2+y^2-2*l^2)/(2*l^2));
k1=l+l*cos(theta2);
k2=l*sin(theta2);
theta1=atan2(y,x)-atan2(k2,k1);