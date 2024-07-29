close all;

A=load('template1.txt');
x=2:0.0001:(2.2-0.0001*1401);
y1=A(1:600,1)/1000;
y2=A(1:600,2)/1000;
b=max(y2);
c=min(y2);
y3=A(1:600,3)/1000;
%y4=A([0 1000],4)/1000;
% figure(1)
% plot(x,y1);
% hold on;
% plot(x,y2);
% plot(x,y3);
% title('Feedback Current Of 2 Phase');
% xlabel('Time [ms]');
% ylabel('Feedback Current [rad]');
% legend('Iq [A]','Ia [A]','Ib [A]');
% grid on;
max2=max(y2); min2=min(y2);
if (max2>-min(y2))
y2_shift=y2-(max2+min2)/2;
else
y2_shift=y2+(max2+min2)/2;
end
y2_shift=y2_shift/max2*3.3;

max3=max(y3); min3=min(y3);
if (max3>-min(y3))
y3_shift=y3-(max3+min3)/2;
else
y3_shift=y3+(max3+min3)/2;
end
y3_shift=y3_shift/max3*3.3;

figure(2)
plot(x,y1, '-r', 'LineWidth', 0.8);
hold on;
plot(x,y2_shift, '-b', 'LineWidth', 0.8);
plot(x,y3_shift, '-black', 'LineWidth', 0.8);
title('Dòng điện phản hồi của hai dây pha đo từ cảm biến dòng');
xlabel('Thời gian [s]');
ylabel('Dòng điện [A]');
legend('Iq tham chiếu [A]','Ia phản hồi [A]','Ib phản hồi [A]');
grid on;

hold off;
