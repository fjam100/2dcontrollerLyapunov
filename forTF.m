Kp=1;Kd=1;K2=1;Ki=1;
m=1;
sys=tf([Kd 0],[Kd*m, m*K2*Kd+m*Kp, Ki*m+m*K2*Kp, K2*Ki*m]);
% bode(sys)
Ki=0;
sys2=tf([Kd 0],[Kd*m, m*K2*Kd+m*Kp, Ki*m+m*K2*Kp, K2*Ki*m]);
bode(sys2);