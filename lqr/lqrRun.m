T=0.01;
Tstart=0;
Tend=10;
times=(Tstart:T:Tend);
M=[1 0; 0 0.3];
C=[1 0; 0 1];
A=[zeros(2,2), eye(2); zeros(2,2) -M\C];
Ad=(eye(size(A))+A*T);
B=[zeros(2,2); inv(M)];
Bd=T*B;

Q=eye(4);
R=eye(2);

[K,S,e]=lqrd(Ad,Bd,Q,R, T);


U=-K*[0;1;0;0]