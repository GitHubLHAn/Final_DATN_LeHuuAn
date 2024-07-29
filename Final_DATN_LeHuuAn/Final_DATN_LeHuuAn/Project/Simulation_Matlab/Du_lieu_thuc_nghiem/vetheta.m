close all;

A=load('theta.txt');

%x1=0:0.00015:(0.09-0.00015);
x1=0:0.00024:(0.144-0.00024);
y1=A(1:600,1)/1000;
y2=A(1:600,2)/1000;

x2=0.356:0.00024:(0.5-0.00024);
y3=A(801:1400,1)/1000;
y4=A(808:1407,2)/1000;

figure()

subplot(2,1,1)
plot(x1,y1, '-r', 'LineWidth', 0.8);
hold on;
plot(x1,y2, '-b', 'LineWidth', 0.8);
xticks([0 0.02 0.04 0.06 0.08 0.1 0.12 0.14]);
xlim([0 0.144]);
ylim([0, 7]);
title('Góc từ thông (Theta) trong quá trình quá độ');
xlabel('Thời gian [s]');
ylabel('Theta [rad]');
legend('Theta thực tế [rad]','Theta ước lượng [rad]');
grid on;

subplot(2,1,2)
plot(x2,y3, '-r', 'LineWidth', 0.8);
hold on;
plot(x2,y4, '-b', 'LineWidth', 0.8);
xticks([0.36 0.38 0.4 0.42 0.44 0.46 0.48 0.5]);
ylim([0, 7]);
xlim([0.356 0.5]);
title('Góc từ thông (Theta) ở trạng thái xác lập');
xlabel('Thời gian [s]');
ylabel('Theta [rad]');
legend('Theta thực tế [rad]','Theta ước lượng [rad]');
grid on;


