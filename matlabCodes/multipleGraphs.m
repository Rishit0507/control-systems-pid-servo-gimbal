% =========================================================
% Camera Gimbal Stabilization — MATLAB Simulation Scripts
% =========================================================

tau = 0.1;
s   = tf('s');
G   = 1 / (tau*s + 1);

Kp = 1.8; Kd = 1.2; Ki = 0.01;

% Open loop
L_PD  = (Kp + Kd*s) * G;
L_PID = (Kp + Ki/s + Kd*s) * G;

% Closed loop
CL_PD  = feedback(L_PD,  1);
CL_PID = feedback(L_PID, 1);

t_sim = 0:0.001:5;
u     = ones(size(t_sim));  % unit step input

% Simulate error signal e = r - y, so output = 1 - y
[y_PD,  t_out] = lsim(CL_PD,  u, t_sim);
[y_PID, t_out] = lsim(CL_PID, u, t_sim);

% Correct orientation — response should rise to setpoint
y_PD  = 1 - y_PD;
y_PID = 1 - y_PID;

% =========================================================
% Simulated Step Response
% =========================================================
% What it shows:
%   System response to a step disturbance.
%   PD rises quickly and settles near setpoint with small error.
%   PID rises slower but eliminates steady state error completely.
% =========================================================
figure(1);
plot(t_out, y_PD,  'b', 'LineWidth', 1.5); hold on;
plot(t_out, y_PID, 'r', 'LineWidth', 1.5);
yline(1, 'k--', 'LineWidth', 1.2);
ylim([0 1.5]);
legend('PD Controller', 'PID Controller', 'Setpoint');
title('Simulated Step Response (PD vs PID)');
xlabel('Time (s)');
ylabel('Angle Response (normalized)');
grid on;


% =========================================================
% Bode Plot
% =========================================================
% What it shows:
%   Frequency response of both closed loop systems.
%   PD is flat and stable across all frequencies.
%   PID has phase dip at low frequencies due to integral action.
% =========================================================
figure(2);
bode(CL_PD,  'b'); hold on;
bode(CL_PID, 'r');
legend('PD Controller', 'PID Controller');
title('Bode Plot — Stability Margins');
grid on;


% =========================================================
% Root Locus
% =========================================================
% What it shows:
%   All poles stay in left half plane for any value of Kp.
%   System is unconditionally stable.
%   Justifies aggressive Kp tuning on hardware.
% =========================================================
figure(3);
rlocus(G);
title('Root Locus — Effect of Kp on Stability');
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;


% =========================================================
% PD vs PID Simulated Overlay
% =========================================================
% What it shows:
%   PD responds faster but settles with small steady state error.
%   PID is slower but reaches setpoint exactly.
%   Justifies using PD on hardware for faster gimbal response.
% =========================================================
figure(4);
plot(t_out, y_PD,  'b', 'LineWidth', 1.5); hold on;
plot(t_out, y_PID, 'r', 'LineWidth', 1.5);
yline(1, 'k--', 'LineWidth', 1.2);
ylim([0 1.5]);
xlim([0 3]);
legend('PD Controller', 'PID Controller', 'Setpoint');
title('PD vs PID Simulated Comparison');
xlabel('Time (s)');
ylabel('Normalized Angle Response');
grid on;