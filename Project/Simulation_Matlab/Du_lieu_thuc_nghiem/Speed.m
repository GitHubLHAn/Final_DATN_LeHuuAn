close all;

A=load('new_spd1.txt');
x=0:0.0015:(3-0.0015*1);
y1=A(1:2000,1);
y2=A(1:2000,2);
figure(1)
plot(x,y1, '-r', 'LineWidth', 0.8);
hold on;
plot(x,y2, '-b', 'LineWidth', 0.8);
ylim([0 150]);
yticks([0 30 60 90 120 150]);
title('Kết Quả Mạch Vòng Tốc Độ Khi Quá Độ');
xlabel('Thời gian [s]');
ylabel('Tốc độ [v/p]');
legend('Tốc độ đặt [v/p]','Tốc độ ước lượng [v/p]');
grid on;
% x2=0:0.002:(2.5-0.002*1);
% y3=A(1:1250,1);
% y4=A(1:1250,2);
% figure(2)
% plot(x2,y3, '-r', 'LineWidth', 0.8);
% hold on;
% plot(x2,y4, '-b', 'LineWidth', 0.8);
% title('Close Speed Loop');
% xlabel('Time [s]');
% ylabel('Speed [RPM]');
% legend('Speed SetPoint [RPM]','Speed Feedback [RPM]');
% grid on;
