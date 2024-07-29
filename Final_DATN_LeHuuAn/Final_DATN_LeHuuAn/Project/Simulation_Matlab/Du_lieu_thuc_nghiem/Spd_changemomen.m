close all;

%A=load('momen_change4.txt');
%A=load('Iq_changemomen.txt');
A=load('new_ch_m3.txt');

x=0:0.002:(4.0-0.002*1);
y1=A(1:2000,1);
y2=A(1:2000,2);
y3=A(1:2000,3)/1000;

figure(1)

%subplot(2,1,1)
plot(x,y1, '-r', 'LineWidth', 0.8);
hold on;
plot(x,y2, '-b', 'LineWidth', 0.8);
ylim([0 150]);
yticks([0 30 60 90 120 150]);
title('Phản ứng của mạch vòng tốc độ khi có tải');
xlabel('Thời gian [s]');
ylabel('Tốc độ [v/p]');
legend('Tốc độ đặt [v/p]','Tốc độ ước lượng [v/p]');
grid on;

figure(2)
%subplot(2,1,2)
plot(x,y3, '-r', 'LineWidth', 0.8);
hold on;
title('Dòng điện tham chiếu Iq khi có tải');
xlabel('Thời gian [s]');
ylabel('Iq [A]');
ylim([0,5]);
legend('Iq tham chiếu [A]');
grid on;
