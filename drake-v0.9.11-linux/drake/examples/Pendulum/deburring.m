checkDependency('spotless');
% checkDependency('mosek');

% set up state variables
x = msspoly('x',1);
y = msspoly('y',1);
xd = msspoly('xd',1);
yd = msspoly('yd',1);
c = msspoly('c',1);
s = msspoly('s',1);
den = msspoly('den',1);
vars = [x;y;xd;yd;c;s;den];

% polynomial dynamics (using parameters from the plant class) in terms of
% s,c, and thetadot


f = [ xd;yd;xdd;ydd;-s*thetad; c*thetad; vard];

prog = spotsosprog;
prog = prog.withIndeterminate(vars);

deg_V = 4;
[prog,V] = prog.newSOSPoly(monomials(vars,0:deg_V));
Vdot = diff(V,vars)*f; 

deg_lambda = 2;
lambda_monom = monomials(vars,0:deg_lambda);
[prog,lambda] = prog.newFreePoly(lambda_monom);


%prog = prog.withSOS( -Vdot - lambda*(s^2+c^2-1) );  % asympotic stability
prog = prog.withSOS( -Vdot - lambda*(s^2+c^2-1) );  % exponential stability
% note: thought I might need s^2*Vdot but didn't actually need to leave out the
% upright

prog = prog.withEqs( subs(V,vars,[0;1;0]) );  % V(0) = 0

% solver = @spot_mosek;
solver = @spot_sedumi;
options = spot_sdp_default_options();
options.verbose = 1;

sol = prog.minimize(0,solver,options);

V = sol.eval(V)
Vdot = sol.eval(Vdot);
