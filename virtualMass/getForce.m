function F=getForce(Y,model)

[m,~]=size(Y);
for i=1:m
    x=Y(i,1);
    y=Y(i,2);
    theta=atan2(-model.spPos(2)+y,-model.spPos(1)+x);
    F(i)=norm(model.spK*((model.r+model.spRad)-norm(model.spPos-[x;y]))*...
        [cos(theta);sin(theta)]);
end