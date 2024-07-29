close all;

A=load('2-Ia.txt');
B=load('2-Ib.txt');
x=2:0.0001:(2.1-0.0001);
y1=A(1:1000,1)/1000;
y2=A(18:1017,2)/1000;
y3=B(1:1000,1)/1000;
y4=B(18:1017,2)/1000;
figure(1)
plot(x,y1);
hold on;
plot(x,y2);

 figure(2)
 plot(x,y3);
 hold on;
 plot(x,y4);


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

figure(3)
plot(x,y1);
hold on;
plot(x,y2_shift);
grid on;

figure(4)
plot(x,y3);
hold on;
plot(x,y4_shift);
grid on;
