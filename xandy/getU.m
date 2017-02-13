function U=getU(X,Xr,model)
Kp=[1 0;0 1];
Kd=[1 0;0 1];
Ks=[3 0;0 3];

x=X(1);
y=X(2);
theta=atan2(-model.spPos(2)+y,-model.spPos(1)+x);

M=[model.mx 0; 0 model.my];

S=Kp*(Xr(1:2)-X(1:2));
U=-inv(Kd)*(-inv(M)*Ks*(Kp*([Xr(1:2)]-[X(1:2)])+Kd*(Xr(3:4)-X(3:4)))-Kp*(Xr(3:4)-X(3:4)))+ ...
    Xr(5:6)+[1/model.mx*model.cx*X(3); 1/model.my*model.cy*X(4)];