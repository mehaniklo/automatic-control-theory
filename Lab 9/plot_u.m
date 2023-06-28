grid minor;
hold on;

t = out.e.Time;
e = out.e.Data(:,4);
plot(t, e);

legend('e4');
