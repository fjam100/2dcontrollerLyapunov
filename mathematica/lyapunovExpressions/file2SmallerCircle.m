epsilon=[((-1).*y+yr).*cos(theta)+(x+(-1).*xr).*sin(theta),(x+(-1).*xr).*cos( ...
  theta)+(y+(-1).*yr).*sin(theta)];
epsilond=[(thetad.*(x+(-1).*xr)+(-1).*yd+yrd).*cos(theta)+(xd+(-1).*xrd+thetad.* ...
  y+(-1).*thetad.*yr).*sin(theta),(xd+(-1).*xrd+thetad.*y+(-1).*thetad.* ...
  yr).*cos(theta)+(thetad.*((-1).*x+xr)+yd+(-1).*yrd).*sin(theta)];
coeffmat=[sin(theta),(-1).*cos(theta);cos(theta),sin(theta)];
rem=[(thetadd.*(x+(-1).*xr)+2.*thetad.*(xd+(-1).*xrd)+cy.*yd+thetad.^2.*(y+( ...
  -1).*yr)+yrdd).*cos(theta)+(-1).*(cx.*xd+thetad.^2.*(x+(-1).*xr)+xrdd+( ...
  -1).*thetadd.*y+thetadd.*yr+(-2).*thetad.*(yd+(-1).*yrd)).*sin(theta),( ...
  -1).*(cx.*xd+thetad.^2.*(x+(-1).*xr)+xrdd+(-1).*thetadd.*y+thetadd.*yr+( ...
  -2).*thetad.*(yd+(-1).*yrd)).*cos(theta)+(-1).*(thetadd.*(x+(-1).*xr)+ ...
  2.*thetad.*(xd+(-1).*xrd)+cy.*yd+thetad.^2.*(y+(-1).*yr)+yrdd).*sin( ...
  theta)];
