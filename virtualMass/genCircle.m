%% Fucntion to generate circle
function [circX, circY]=genCircle(center,rad,Ts,thetad)
%center=[x1,y1] rad=radius (all in mm)
T=2*pi/thetad;
t=(0:Ts:T);
theta=t/T*2*pi;

circX=center(1)+rad*cos(theta);
circY=center(2)+rad*sin(theta);
