close all;

A=load('2-Ia.txt');
B=load('2-Ib.txt');

% A=load('run_ia.txt');
% B=load('run_ib.txt');
x=0.1:0.0001:(0.2-0.0001);
y1=A(1:1000,1)/1000;
y2=A(18:1017,2)/1000;
y3=B(1:1000,1)/1000;
y4=B(18:1017,2)/1000;


max2=max(y2); min2=min(y2);
if (max2>-min(y2))
y2_shift=y2-(max2+min2)/2;
else
y2_shift=y2+(max2+min2)/2;
end
y2_shift=y2_shift/max2*3.3;

max4=max(y4); min4=min(y4);
if (max4>-min(y4))
y4_shift=y4-(max4+min4)/2;
else
y4_shift=y4+(max4+min4)/2;
end
y4_shift=y4_shift/max4*3.3;

figure()

subplot(2,1,1)
plot(x,y1, '-r', 'LineWidth', 0.8);
hold on;
plot(x,y2_shift, '-b', 'LineWidth', 0.8);
yticks([-3 -1.5 0 1.5 3]);
title('Dòng điện pha A');
xlabel('Thời gian [s]');
ylabel('Dòng điện [A]');
legend('Ia tham chiếu [A]','Ia phản hồi [A]');
grid on;

subplot(2,1,2)
plot(x,y3, '-r', 'LineWidth', 0.8);
hold on;
plot(x,y4_shift, '-b', 'LineWidth', 0.8);
yticks([-3 -1.5 0 1.5 3]);
title('Dòng điện pha B');
xlabel('Thời gian [s]');
ylabel('Dòng điện [A]');
legend('Ib tham chiếu [A]','Ib phản hồi [A]');
grid on;
