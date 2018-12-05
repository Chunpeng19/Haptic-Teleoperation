input_matrix = load("/home/lowell/Documents/Haptic-Teleoperation/examples/systemidentification/joint1DataX5.txt");
time = input_matrix(:,1);
pos = input_matrix(:,2);
pos = abs(pos - 0.4775 * ones(size(pos,1),1));
f = @(b,time) b(1)-b(1).*exp(b(2)*time);
B = fminsearch(@(b) norm(pos-f(b,time)),[0.3;-8.0]);
plot(time,pos,'r*')
hold on
plot(time,f(B,time),'b.')
hold off
grid on
xlabel('time (s)')
ylabel('angle (rad)')
title('joint angle response')
legend('raw data','fit curve')
text(0.4,0.15, sprintf('x(t)=%.3f(1-e^{%.3f\\cdott})',B))