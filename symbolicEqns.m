syms theta kt kn ct cn x y xd yd;
T=[-sin(theta) -cos(theta); cos(theta) -sin(theta)];
Tinv=inv(T);
Tinvd=diff(Tinv,theta);
term1=T*([kt 0;0 kn]*Tinv +[ct 0;0 cn]*Tinvd);

term2=T*[ct 0; 0 cn]*Tinv;

term1Subs=subs(term1,[kt,kn,ct,cn],[10000, 100, 1, 10]);

negativeDef=0;
for thetaVal=0:0.1:1
    term=subs(term1Subs,theta,thetaVal);
end