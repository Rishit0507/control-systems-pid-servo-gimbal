load('gimbal_data.mat');

figure(1);
subplot(2,1,1);
plot(t, pitch, 'b', 'LineWidth', 1.5); hold on;
plot(t, roll, 'r', 'LineWidth', 1.5);
yline(0, 'k--', 'Setpoint');
xlabel('Time (s)'); ylabel('Angle (degrees)');
title('Step Response — Pitch and Roll');
legend('Pitch', 'Roll', 'Setpoint');
grid on;

subplot(2,1,2);
plot(t, pitchCmd, 'b', 'LineWidth', 1.5); hold on;
plot(t, rollCmd, 'r', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Servo Command (degrees)');
title('Servo Commands');
legend('Pitch CMD', 'Roll CMD');
grid on;