grid minor;
hold on;

t = out.x.Time;
x = out.x.Data(:,1);
plot(t, x);
t_ = out.x_.Time;
x_ = out.x_.Data(:,1);
plot(t_, x_);

legend('x1', 'x1hat');
