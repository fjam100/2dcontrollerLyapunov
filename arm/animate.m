function animate(Y,model,Xr)
for i=1:length(Y(:,1))
   X0=[0;0];
   X1=[model.l*cos(Y(i,1));model.l*sin(Y(i,1))];
   X2=X1+[model.l*cos(Y(i,1)+Y(i,2)); model.l*sin(Y(i,1)+Y(i,2))];
   data=[X0,X1,X2];
   xdata=data(1,:);
   ydata=data(2,:);
   clf;
   
   h=line('XData',xdata,'YData',ydata);
   hold on;
   axis('equal');
   plot(Xr(:,1),Xr(:,2),'o');
   ylim([0,2]);
   pause(0.001);
end