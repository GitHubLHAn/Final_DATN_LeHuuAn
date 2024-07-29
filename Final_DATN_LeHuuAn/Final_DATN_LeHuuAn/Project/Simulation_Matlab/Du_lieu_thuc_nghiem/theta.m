close all;

A=load('theta.txt');
x=0:0.00015:(0.3-0.00015*1001);
y1=A(1:1000,1)/1000;
y2=A(1:1000,2)/1000;
figure(1)
plot(x,y1);
hold on;
plot(x,y2);
title('Feedback Current Of 2 Phase');
xlabel('Time [ms]');
ylabel('Feedback Current [rad]');
legend('theta_real [A]','theta_est [A]');
grid on;
x2=0.2:0.00015:(0.2+0.00015*599);
y3=A(801:1400,1)/1000;
y4=A(808:1407,2)/1000;
figure(2)
plot(x2,y3);
hold on;
plot(x2,y4);
grid on;


