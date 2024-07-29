close all;

A=load('est_spd.txt');
x=0:0.002:(4-0.002*1);
y1=A(1:2000,1);
y2=A(1:2000,2);
figure(1)
plot(x,y1, '-r', 'LineWidth', 0.8);
hold on;
plot(x,y2, '-b', 'LineWidth', 0.8);
ylim([0, 80]);
title('So sánh tốc độ đo từ Encoder và tốc độ ước lượng từ bộ PLL');
xlabel('Thời gian [s]');
ylabel('Tốc độ [v/p]');
legend('Tốc độ đo [v/p]','Tốc độ ước lượng [v/p]');
grid on;