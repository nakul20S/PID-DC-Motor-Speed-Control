%% PID-Controlled DC Motor Speed Control
%  Author      : Nakul S
%  Tool        : MATLAB R2024a + Control System Toolbox
%  Description : Models a DC motor as a 2nd-order transfer function,
%                designs a PID controller, and analyses performance
%                through step response, disturbance rejection, Bode plot,
%                and root locus.
% =========================================================================

clc; clear; close all;

%% ── 1. MOTOR PARAMETERS ─────────────────────────────────────────────────
% Derived from armature (electrical) and rotor (mechanical) equations:
%   Electrical: V = L·di/dt + R·i + Ke·ω
%   Mechanical: J·dω/dt = Kt·i − B·ω

R  = 1;      % Armature resistance     (Ω)
L  = 0.5;    % Armature inductance     (H)
Kt = 0.01;   % Torque constant         (N·m/A)
Ke = 0.01;   % Back-EMF constant       (V·s/rad)
J  = 0.01;   % Rotor inertia           (kg·m²)
B  = 0.1;    % Viscous friction coeff  (N·m·s/rad)

%% ── 2. PLANT TRANSFER FUNCTION ──────────────────────────────────────────
% After Laplace transform and algebraic manipulation:
%   G(s) = Kt / [(Js+B)(Ls+R) + Ke·Kt]
%
% Denominator coefficients:
%   s² : J·L
%   s¹ : J·R + B·L
%   s⁰ : B·R + Ke·Kt

num = Kt;
den = [(J*L), (J*R + B*L), (B*R + Ke*Kt)];

G = tf(num, den);
fprintf('Plant transfer function G(s):\n'); disp(G)

% DC gain — steady-state speed per unit voltage, no controller
DC_gain = Kt / (B*R + Ke*Kt);
fprintf('DC gain (open-loop):  %.4f rad/s per volt\n', DC_gain);

%% ── 3. OPEN-LOOP STEP RESPONSE ──────────────────────────────────────────
figure(1);
step(G);
title('Open-loop DC Motor Step Response');
xlabel('Time (s)'); ylabel('Speed (rad/s)'); grid on;

%% ── 4. PID CONTROLLER DESIGN ────────────────────────────────────────────
% Tuning logic:
%   Kp : increase until fast rise, no excessive overshoot
%   Ki : increase until steady-state error → 0
%   Kd : increase to suppress overshoot from integral action

Kp = 100;
Ki = 200;
Kd = 10;

C_P   = pid(Kp, 0,  0 );    % Proportional only
C_PI  = pid(Kp, Ki, 0 );    % Proportional + Integral
C_PID = pid(Kp, Ki, Kd);    % Full PID

%% ── 5. CLOSED-LOOP TRANSFER FUNCTIONS ──────────────────────────────────
T_P   = feedback(C_P   * G, 1);
T_PI  = feedback(C_PI  * G, 1);
T_PID = feedback(C_PID * G, 1);

%% ── 6. CONTROLLER COMPARISON PLOT ──────────────────────────────────────
SP = 10;    % Setpoint: 10 rad/s

figure(2);
step(SP*T_P, SP*T_PI, SP*T_PID);
legend('P only', 'PI', 'Full PID', 'Location', 'southeast');
title('PID Tuning Comparison — Same Plant, Three Controllers');
xlabel('Time (s)'); ylabel('Speed (rad/s)'); grid on;

%% ── 7. PERFORMANCE METRICS ──────────────────────────────────────────────
info = stepinfo(SP * T_PID);
fprintf('\n--- PID Performance Metrics ---\n');
fprintf('Rise Time:      %.3f s\n',   info.RiseTime);
fprintf('Settling Time:  %.3f s\n',   info.SettlingTime);
fprintf('Overshoot:      %.2f %%\n',  info.Overshoot);
fprintf('Peak:           %.4f rad/s\n', info.Peak);

%% ── 8. DISTURBANCE REJECTION ────────────────────────────────────────────
% Load torque disturbance injected at t = 5s
% Disturbance-to-output TF: D(s) = feedback(G, C_PID)

figure(3);
t = 0:0.01:10;
[y1, t1] = step(SP * T_PID, t);
D_tf     = feedback(G, C_PID);
[yd, ~]  = step(D_tf, t);
y_total  = y1 - yd;

plot(t1, y_total, 'b', 'LineWidth', 1.5);
xline(5, '--r', 'Disturbance injected', 'LabelVerticalAlignment', 'bottom');
title('Closed-Loop with Load Disturbance Rejection');
xlabel('Time (s)'); ylabel('Speed (rad/s)'); grid on;
ylim([0 12]);

%% ── 9. BODE PLOT — STABILITY MARGINS ───────────────────────────────────
figure(4);
margin(C_PID * G);
title('Open-Loop Bode Plot with Gain and Phase Margin');
grid on;

[Gm, Pm, Wgm, Wpm] = margin(C_PID * G);
fprintf('\n--- Stability Margins ---\n');
fprintf('Gain Margin:   %.2f dB  at %.3f rad/s\n', 20*log10(Gm), Wgm);
fprintf('Phase Margin:  %.2f deg at %.3f rad/s\n', Pm, Wpm);

%% ── 10. ROOT LOCUS ──────────────────────────────────────────────────────
figure(5);
rlocus(G);
title('Root Locus — DC Motor Plant');
grid on;

%% ── 11. SAVE ALL FIGURES ────────────────────────────────────────────────
if ~exist('figures', 'dir'), mkdir('figures'); end
saveas(figure(1), 'figures/P1_openloop.png');
saveas(figure(2), 'figures/P1_comparison.png');
saveas(figure(3), 'figures/P1_disturbance.png');
saveas(figure(4), 'figures/P1_bode.png');
saveas(figure(5), 'figures/P1_rootlocus.png');
fprintf('\nAll figures saved to /figures/\n');
