[time, goal, y] = importfile('logdata.csv', 30, inf);
clf; hold on;
plot(time - time(1), goal);
plot(time - time(1), y);
legend('setpoint', 'state');