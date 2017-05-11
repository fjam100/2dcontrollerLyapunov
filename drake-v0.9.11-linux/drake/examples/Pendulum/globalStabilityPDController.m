function globalStabilityPDController

% Use SOS to find a Lyapunov function to certify the global stability of
% the damped pendulum.  Uses trig to poly substitution to perform exact
% analysis.  Recovers a Lyapunov function that looks a lot like the energy,
% but is better!  (doesn't require LaSalle's theorem)

checkDependency('spotless');
% checkDependency('mosek');

% set up state variables
% syms x y xd yd c s theta r;
x=msspoly('x',1);
y=msspoly('y',1);
xd=msspoly('xd',1);
yd=msspoly('yd',1);
s = msspoly('s',1);
c = msspoly('c',1);
theta=msspoly('th',1);
r=msspoly('r',1);
vars = [x; y; xd; yd; c; s; theta; r];

% polynomial dynamics (using parameters from the plant class) in terms of
% s,c, and thetadot
xr=0;yr=0;
M=[1,0;0,0.3];
C=[1 0; 0 1];
K=[100, 0; 0 ,1];
% Constructing the vector field dx/dt = f
epsilont=dot([xr - x; yr - y], [-s; c]);
epsilonn=dot([xr-x; yr-y],[-c;-s]);
T=[-s -c;c -s];
Xdd=inv(M)*(T*K*[epsilont;epsilonn]-C*[xd;yd]);
xdd=Xdd(1);
ydd=Xdd(2);
thetad=(-xd*y+yd*x)*r;
rd=(x*xd+y*yd)*r^3;

f = [xd;yd;xdd;ydd;-s*thetad;c*thetad;thetad;rd];

prog = spotsosprog;
prog = prog.withIndeterminate(vars);

deg_V = 4;
[prog,V] = prog.newSOSPoly(monomials(vars,0:deg_V));
Vdot = diff(V,vars)*f; 

deg_lambda = 2;
lambda_monom = monomials(vars,0:deg_lambda);
[prog,lambda] = prog.newFreePoly(lambda_monom);


%prog = prog.withSOS( -Vdot - lambda*(s^2+c^2-1) );  % asympotic stability
prog = prog.withSOS( -Vdot - lambda*(s^2+c^2-1));  % exponential stability
% note: thought I might need s^2*Vdot but didn't actually need to leave out the
% upright

% prog = prog.withEqs( subs(V,vars,[0;1;0]) );  % V(0) = 0

% solver = @spot_mosek;
solver = @spot_sedumi;
options = spot_sdp_default_options();
options.verbose = 1;

sol = prog.minimize(0,solver,options);

V = sol.eval(V)
Vdot = sol.eval(Vdot)
% 
% 
% [Theta,ThetaDot] = meshgrid(-pi:0.1:pi, -8:0.25:8);  % for surfs
% 
% % first plot contours
% 
% figure(1);
% subplot(1,2,1);
% ezcontour(@(theta,thetadot)dmsubs(V,vars,[sin(theta');cos(theta');thetadot']),[-2*pi,2*pi,-8,8]);
% title('$$ V $$','interpreter','latex','fontsize',20) 
% xlabel('$$ \theta $$','interpreter','latex','fontsize',15)
% ylabel('$$ \dot{\theta} $$','interpreter','latex','fontsize',15)
% subplot(1,2,2);
% Vmesh = dmsubs(V,vars,[sin(Theta(:)');cos(Theta(:)');ThetaDot(:)']);
% surf(Theta,ThetaDot,reshape(Vmesh,size(Theta)));
% title('$$ V $$','interpreter','latex','fontsize',20) 
% xlabel('$$ \theta $$','interpreter','latex','fontsize',15)
% ylabel('$$ \dot{\theta} $$','interpreter','latex','fontsize',15)
% 
% figure(2);
% subplot(1,2,1);
% ezcontour(@(theta,thetadot)dmsubs(Vdot,vars,[sin(theta');cos(theta');thetadot']),[-2*pi,2*pi,-8,8]);
% title('$$ \dot{V} $$','interpreter','latex','fontsize',20) 
% xlabel('$$ \theta $$','interpreter','latex','fontsize',15)
% ylabel('$$ \dot{\theta} $$','interpreter','latex','fontsize',15)
% subplot(1,2,2);
% Vdotmesh = dmsubs(Vdot,vars,[sin(Theta(:)');cos(Theta(:)');ThetaDot(:)']);
% surf(Theta,ThetaDot,reshape(Vdotmesh,size(Theta)));
% title('$$ \dot{V} $$','interpreter','latex','fontsize',20) 
% xlabel('$$ \theta $$','interpreter','latex','fontsize',15)
% ylabel('$$ \dot{\theta} $$','interpreter','latex','fontsize',15)
% 
end
