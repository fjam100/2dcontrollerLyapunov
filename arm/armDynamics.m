function res=armDynamics(t,y,model,tau)
theta1 = y(1);
theta2 = y(2);
theta1d = y(3);
theta2d = y(4);

g=0;
m1 = model.m1;
m2 = model.m2;
l = model.l;

J = [-l*sin(theta1), -l*sin(theta1+theta2); l*cos(theta1) + l*cos(theta1+theta2), l*cos(theta1+theta2)];

X = [l*cos(theta1) + l*cos(theta1+theta2); l*sin(theta1)+ l*sin(theta1+theta2)];
xact=X(1);
yact=X(2);
Fspindle = [0;0];

if (model.r+model.addRad)>norm(model.spPos-[xact;yact])
    alpha=atan2(-model.spPos(2)+yact,-model.spPos(1)+xact);
    Fspindle=model.spK*((model.r+model.addRad)-norm(model.spPos-[xact;yact]))*...
        [cos(alpha);sin(alpha)];
%     Fnoise=rand(2,1)*0.1;
end


Tauext=J.'*Fspindle;
Hmat = [1/3*m1*l^2 + 4/3*m2*l^2 + m2*cos(theta2)*l^2,1/3*m2*l^2 + 1/2*m2*cos(theta2)*l^2; ...
    1/3*m2*l^2 + 1/2*m2*cos(theta2)*l^2, 1/3*m2*l^2];
Cvec = [-1/2*m2*sin(theta2)*l^2*theta2d^2 - ...
     m2*sin(theta2)*l^2*theta1d*theta2d; 1/2*m2*sin(theta2)*l^2* ...
     theta1d^2];
Gvec = [(1/2*m1 + m2)*g*l*cos(theta1) +  ...
     1/2*m2*g*l*cos(theta1 + theta2); 1/2*m2*g*l* ...
     cos(theta1 + theta2)];

res = [Hmat\(tau - Cvec - Gvec + Tauext);y(3);y(4)];