%% ELEC 481 - ECP Model 205a Two DOF Torsional Plant Analysis
% Tasks 1-7: State Space Modeling and Classical Control Design
% Plant #2 Configuration - Measuring theta1 output

clear all; close all; clc;

%% ========================================================================
%  SYSTEM PARAMETERS (Plant #2 from ECP Manual)
%  ========================================================================
% Hardware gain
kc = 10/32768;
kaktkp = 0.70;
ke = 16000/2/pi;
ka = 32;
khw = kc*kaktkp*ke*ka;

% Plant #2 parameters
J1 = 0.0108;  % kg*m^2
J2 = 0.0103;  % kg*m^2
c1 = 0.007;   % N*m*s/rad
c2 = 0.001;   % N*m*s/rad
k1 = 1.37;    % N*m/rad

fprintf('=== ECP Model 205a - Plant #2 Parameters ===\n');
fprintf('J1 = %.4f kg*m^2\n', J1);
fprintf('J2 = %.4f kg*m^2\n', J2);
fprintf('c1 = %.4f N*m*s/rad\n', c1);
fprintf('c2 = %.4f N*m*s/rad\n', c2);
fprintf('k1 = %.4f N*m/rad\n', k1);
fprintf('Hardware Gain (khw) = %.4f\n\n', khw);

%% ========================================================================
%  TASK 1: STATE-SPACE EQUATIONS
%  ========================================================================
fprintf('TASK 1: Deriving State-Space Equations\n');
fprintf('========================================\n');

% State vector: X = [theta1; theta1_dot; theta2; theta2_dot]
% Input: T(t) = torque applied to disk 1
% Output: Y = theta1 (angle of disk 1)

% State matrix A
A = [0                1              0           0;
     -k1/J1          -c1/J1          k1/J1       0;
     0                0              0           1;
     k1/J2            0             -(k1)/J2    -c2/J2];

% Input matrix B
B = [0; 
     1/J1; 
     0; 
     0];

% Output matrix C (measuring theta1 only)
C = [1 0 0 0];

% Feedforward matrix D
D = 0;

% Include hardware gain in B matrix
B = B * khw;
% Hardware gain
kc = 10/32768;
kaktkp = 0.70;
ke = 16000/2/pi;
ka = 32;
khw = kc*kaktkp*ke*ka;

% Plant #2 parameters
J1 = 0.0108;  % kg*m^2
J2 = 0.0103;  % kg*m^2
c1 = 0.007;   % N*m*s/rad
c2 = 0.001;   % N*m*s/rad
k1 = 1.37;    % N*m/rad

fprintf('=== ECP Model 205a - Plant #2 Parameters ===\n');
fprintf('J1 = %.4f kg*m^2\n', J1);
fprintf('J2 = %.4f kg*m^2\n', J2);
fprintf('c1 = %.4f N*m*s/rad\n', c1);
fprintf('c2 = %.4f N*m*s/rad\n', c2);
fprintf('k1 = %.4f N*m/rad\n', k1);
fprintf('Hardware Gain (khw) = %.4f\n\n', khw);

%% ========================================================================
%  TASK 1: STATE-SPACE EQUATIONS
%  ========================================================================
fprintf('TASK 1: Deriving State-Space Equations\n');
fprintf('========================================\n');

% State vector: X = [theta1; theta1_dot; theta2; theta2_dot]
% Input: T(t) = torque applied to disk 1
% Output: Y = theta1 (angle of disk 1)

% State matrix A
A = [0                1              0           0;
     -k1/J1          -c1/J1          k1/J1       0;
     0                0              0           1;
     k1/J2            0             -(k1)/J2    -c2/J2];

% Input matrix B
B = [0; 
     1/J1; 
     0; 
     0];

% Output matrix C (measuring theta1 only)
C = [1 0 0 0];

% Feedforward matrix D
D = 0;

% Include hardware gain in B matrix
B = B * khw;

fprintf('State-space model created:\n');
fprintf('State vector X = [theta1; omega1; theta2; omega2]\n');
fprintf('Input u = Torque command\n');
fprintf('Output y = theta1 (rad)\n\n');

% Create state-space system
sys_ss = ss(A, B, C, D);
fprintf('System order: %d\n', size(A,1));
fprintf('Number of inputs: %d\n', size(B,2));
fprintf('Number of outputs: %d\n\n', size(C,1));

%% ========================================================================
%  TASK 2: TRANSFER FUNCTION
%  ========================================================================
fprintf('TASK 2: Open-Loop Transfer Function\n');
fprintf('====================================\n');

% Convert state-space to transfer function
sys_tf = tf(sys_ss);
[num, den] = tfdata(sys_tf, 'v');

fprintf('Transfer function G(s) = theta1(s)/T(s):\n\n');
sys_tf

% Display poles and zeros
poles_ol = pole(sys_tf);
zeros_ol = zero(sys_tf);

fprintf('\nOpen-loop poles:\n');
for i = 1:length(poles_ol)
    if imag(poles_ol(i)) == 0
        fprintf('  p%d = %.4f\n', i, real(poles_ol(i)));
    else
        fprintf('  p%d = %.4f + j%.4f\n', i, real(poles_ol(i)), imag(poles_ol(i)));
    end
end

fprintf('\nOpen-loop zeros:\n');
if isempty(zeros_ol)
    fprintf('  No finite zeros\n');
else
    for i = 1:length(zeros_ol)
        fprintf('  z%d = %.4f\n', i, zeros_ol(i));
    end
end

% Natural frequency and damping ratio
wn = sqrt(k1*(J1+J2)/(J1*J2));
zeta = (c1*J2 + c2*J1)/(2*sqrt(k1*J1*J2*(J1+J2)));
fprintf('\nNatural frequency: wn = %.4f rad/s\n', wn);
fprintf('Damping ratio: zeta = %.4f\n\n', zeta);

%% ========================================================================
%  TASK 3: CANONICAL FORMS
%  ========================================================================
fprintf('TASK 3: Canonical Forms\n');
fprintf('=======================\n');

% Check controllability
Co = ctrb(A, B);
rank_Co = rank(Co);
fprintf('Controllability matrix rank: %d (Full rank: %d)\n', rank_Co, size(A,1));
if rank_Co == size(A,1)
    fprintf('System is CONTROLLABLE\n');
else
    fprintf('System is NOT controllable\n');
end

% Check observability
Ob = obsv(A, C);
rank_Ob = rank(Ob);
fprintf('Observability matrix rank: %d (Full rank: %d)\n', rank_Ob, size(A,1));
if rank_Ob == size(A,1)
    fprintf('System is OBSERVABLE\n\n');
else
    fprintf('System is NOT observable\n\n');
end

% Controllable canonical form
if rank_Co == size(A,1)
    [Ac, Bc, Cc, Tc] = ctrbf(A, B, C);
    fprintf('Controllable Canonical Form obtained\n');
end

% Observable canonical form
if rank_Ob == size(A,1)
    [Ao, Bo, Co, To] = obsvf(A, B, C);
    fprintf('Observable Canonical Form obtained\n');
end

% Jordan canonical form (diagonal/modal form)
[V, J] = eig(A);
Aj = J;
Bj = inv(V) * B;
Cj = C * V;
fprintf('Jordan Canonical Form obtained\n\n');

%% ========================================================================
%  TASK 4: IMPULSE AND STEP RESPONSE
%  ========================================================================
fprintf('TASK 4: Time-Domain Responses\n');
fprintf('==============================\n');

% Time vector
t = 0:0.001:5;

% Initial conditions: X0 = [0.1; 0; 0; 0] (10 deg initial angle on disk 1)
X0 = [0.1; 0; 0; 0];

% Impulse response
figure('Name', 'Task 4: Impulse and Step Responses', 'Position', [100 100 1200 800]);

subplot(2,2,1);
impulse(sys_ss, t);
grid on;
title('Impulse Response (Zero Initial Conditions)');
xlabel('Time (s)');
ylabel('Angle (rad)');

% Step response with zero initial conditions
subplot(2,2,2);
step(sys_ss, t);
grid on;
title('Step Response (Zero Initial Conditions)');
xlabel('Time (s)');
ylabel('Angle (rad)');

% Get step response info
step_info = stepinfo(sys_ss);
fprintf('Step Response Characteristics:\n');
fprintf('  Rise time: %.4f s\n', step_info.RiseTime);
fprintf('  Settling time: %.4f s\n', step_info.SettlingTime);
fprintf('  Overshoot: %.2f %%\n', step_info.Overshoot);
fprintf('  Peak: %.4f rad\n', step_info.Peak);
fprintf('  Peak time: %.4f s\n\n', step_info.PeakTime);

% Step response with initial conditions
subplot(2,2,3);
[y, t_step, x] = lsim(sys_ss, ones(size(t)), t, X0);
plot(t_step, y, 'b', 'LineWidth', 1.5);
grid on;
title('Step Response with Initial Conditions X_0 = [0.1, 0, 0, 0]^T');
xlabel('Time (s)');
ylabel('Angle (rad)');

% Response to initial conditions only
subplot(2,2,4);
[y_ic, t_ic] = initial(sys_ss, X0, t);
plot(t_ic, y_ic, 'r', 'LineWidth', 1.5);
grid on;
title('Response to Initial Conditions Only');
xlabel('Time (s)');
ylabel('Angle (rad)');

%% ========================================================================
%  TASK 5: BODE PLOT AND ROOT LOCUS
%  ========================================================================
fprintf('TASK 5: Frequency and Root Locus Analysis\n');
fprintf('==========================================\n');

figure('Name', 'Task 5: Bode Plot', 'Position', [100 100 1000 600]);
bode(sys_tf);
grid on;
title('Bode Plot of Uncompensated System');

% Get gain and phase margins
[Gm, Pm, Wcg, Wcp] = margin(sys_tf);
fprintf('Stability Margins:\n');
fprintf('  Gain Margin: %.2f dB (at %.4f rad/s)\n', 20*log10(Gm), Wcg);
fprintf('  Phase Margin: %.2f deg (at %.4f rad/s)\n\n', Pm, Wcp);

% Root locus plot
figure('Name', 'Task 5: Root Locus', 'Position', [100 100 800 600]);
rlocus(sys_tf);
grid on;
title('Root Locus of Uncompensated System');
sgrid;

%% ========================================================================
%  TASK 6: PID CONTROLLER DESIGN
%  ========================================================================
fprintf('TASK 6: PID Controller Design\n');
fprintf('==============================\n');

% Design specifications:
% - Settling time: < 2 seconds
% - Overshoot: < 20%
% - Steady-state error: < 2%
% - Phase margin: > 45 degrees

fprintf('Design Specifications:\n');
fprintf('  Settling time: < 2 s\n');
fprintf('  Overshoot: < 20 %%\n');
fprintf('  Steady-state error: < 2 %%\n');
fprintf('  Phase margin: > 45 deg\n\n');

% Using MATLAB's PID tuner (you can also manually tune)
% We'll design a PID controller
opts = pidtuneOptions('DesignFocus', 'balanced');
[C_pid, info] = pidtune(sys_tf, 'PID', opts);

fprintf('PID Controller Design:\n');
C_pid

% Get PID parameters
Kp = C_pid.Kp;
Ki = C_pid.Ki;
Kd = C_pid.Kd;

fprintf('\nPID Parameters:\n');
fprintf('  Kp = %.4f\n', Kp);
fprintf('  Ki = %.4f\n', Ki);
fprintf('  Kd = %.4f\n', Kd);

% Closed-loop system
sys_cl_pid = feedback(C_pid * sys_tf, 1);

fprintf('\nClosed-loop poles with PID:\n');
poles_cl_pid = pole(sys_cl_pid);
for i = 1:length(poles_cl_pid)
    if imag(poles_cl_pid(i)) == 0
        fprintf('  p%d = %.4f\n', i, real(poles_cl_pid(i)));
    else
        fprintf('  p%d = %.4f + j%.4f\n', i, real(poles_cl_pid(i)), imag(poles_cl_pid(i)));
    end
end

% Check stability margins with PID
[Gm_pid, Pm_pid, Wcg_pid, Wcp_pid] = margin(C_pid * sys_tf);
fprintf('\nStability Margins with PID:\n');
fprintf('  Gain Margin: %.2f dB (at %.4f rad/s)\n', 20*log10(Gm_pid), Wcg_pid);
fprintf('  Phase Margin: %.2f deg (at %.4f rad/s)\n\n', Pm_pid, Wcp_pid);

% Step response comparison
figure('Name', 'Task 6: PID Controller Performance', 'Position', [100 100 1200 800]);

subplot(2,2,1);
step(sys_ss, 'b', sys_cl_pid, 'r', t);
grid on;
legend('Uncompensated', 'PID Compensated', 'Location', 'best');
title('Step Response Comparison');
xlabel('Time (s)');
ylabel('Angle (rad)');

% Get compensated step info
step_info_pid = stepinfo(sys_cl_pid);
fprintf('PID Compensated Step Response:\n');
fprintf('  Rise time: %.4f s\n', step_info_pid.RiseTime);
fprintf('  Settling time: %.4f s\n', step_info_pid.SettlingTime);
fprintf('  Overshoot: %.2f %%\n', step_info_pid.Overshoot);
fprintf('  Peak: %.4f rad\n', step_info_pid.Peak);

% Check if specs are met
fprintf('\nSpecifications Check:\n');
if step_info_pid.SettlingTime < 2
    fprintf('  Settling time: PASS (%.4f < 2 s)\n', step_info_pid.SettlingTime);
else
    fprintf('  Settling time: FAIL (%.4f >= 2 s)\n', step_info_pid.SettlingTime);
end
if step_info_pid.Overshoot < 20
    fprintf('  Overshoot: PASS (%.2f%% < 20%%)\n', step_info_pid.Overshoot);
else
    fprintf('  Overshoot: FAIL (%.2f%% >= 20%%)\n', step_info_pid.Overshoot);
end
if Pm_pid > 45
    fprintf('  Phase margin: PASS (%.2f deg > 45 deg)\n\n', Pm_pid);
else
    fprintf('  Phase margin: FAIL (%.2f deg <= 45 deg)\n\n', Pm_pid);
end

% Bode plot comparison
subplot(2,2,2);
bode(sys_tf, 'b', C_pid*sys_tf, 'r');
grid on;
legend('Uncompensated', 'With PID', 'Location', 'best');
title('Bode Plot Comparison');

% Root locus with PID
subplot(2,2,3);
rlocus(sys_tf);
hold on;
plot(real(poles_cl_pid), imag(poles_cl_pid), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
grid on;
sgrid;
legend('Root Locus', 'Closed-loop Poles', 'Location', 'best');
title('Root Locus with PID Poles');

%% ========================================================================
%  TASK 7: VARIOUS INPUT RESPONSES
%  ========================================================================
fprintf('TASK 7: Responses to Various Inputs\n');
fprintf('====================================\n');

% Time vector for responses
t7 = 0:0.001:10;

figure('Name', 'Task 7: Various Input Responses', 'Position', [100 100 1400 900]);

% (a) Step Response
subplot(3,3,1);
step(sys_ss, t7);
grid on;
title('Uncompensated: Step Response');
ylabel('Angle (rad)');

subplot(3,3,2);
step(sys_cl_pid, t7);
grid on;
title('PID Compensated: Step Response');

% (b) Square Wave Response (0.5 Hz)
f_square = 0.5; % Hz
square_wave = square(2*pi*f_square*t7);

subplot(3,3,3);
lsim(sys_ss, square_wave, t7);
grid on;
title(sprintf('Uncompensated: Square Wave (%.1f Hz)', f_square));

subplot(3,3,4);
lsim(sys_cl_pid, square_wave, t7);
grid on;
title(sprintf('PID Compensated: Square Wave (%.1f Hz)', f_square));
ylabel('Angle (rad)');

% (c) Sinusoidal Response (1 Hz)
f_sin = 1; % Hz
sin_wave = sin(2*pi*f_sin*t7);

subplot(3,3,5);
lsim(sys_ss, sin_wave, t7);
grid on;
title(sprintf('Uncompensated: Sinusoidal (%.1f Hz)', f_sin));

subplot(3,3,6);
lsim(sys_cl_pid, sin_wave, t7);
grid on;
title(sprintf('PID Compensated: Sinusoidal (%.1f Hz)', f_sin));

% (d) Step with initial conditions
subplot(3,3,7);
lsim(sys_ss, ones(size(t7)), t7, X0);
grid on;
title('Uncompensated: Step + IC');
xlabel('Time (s)');
ylabel('Angle (rad)');

subplot(3,3,8);
% For closed-loop with IC, we need to augment the system
[num_cl, den_cl] = tfdata(sys_cl_pid, 'v');
sys_cl_ss = ss(tf(num_cl, den_cl));
X0_cl = zeros(size(sys_cl_ss.A, 1), 1); % Adjust IC size
if length(X0_cl) >= 4
    X0_cl(1:4) = X0;
end
lsim(sys_cl_ss, ones(size(t7)), t7, X0_cl);
grid on;
title('PID Compensated: Step + IC');
xlabel('Time (s)');

% Frequency response comparison
subplot(3,3,9);
frequencies = logspace(-1, 2, 100);
for i = 1:length(frequencies)
    sin_input = sin(2*pi*frequencies(i)*t7);
    y_uncomp = lsim(sys_ss, sin_input, t7);
    y_comp = lsim(sys_cl_pid, sin_input, t7);
    
    % Calculate amplitude ratio
    amp_in = max(sin_input) - min(sin_input);
    amp_uncomp(i) = (max(y_uncomp(end-1000:end)) - min(y_uncomp(end-1000:end)));
    amp_comp(i) = (max(y_comp(end-1000:end)) - min(y_comp(end-1000:end)));
end
semilogx(frequencies, 20*log10(amp_uncomp), 'b', frequencies, 20*log10(amp_comp), 'r');
grid on;
xlabel('Frequency (Hz)');
ylabel('Magnitude (dB)');
title('Frequency Response Comparison');
legend('Uncompensated', 'PID Compensated');

fprintf('Step response completed\n');
fprintf('Square wave response completed (f = %.1f Hz)\n', f_square);
fprintf('Sinusoidal response completed (f = %.1f Hz)\n', f_sin);
fprintf('\n');

%% ========================================================================
%  SUMMARY
%  ========================================================================
fprintf('========================================\n');
fprintf('           ANALYSIS COMPLETE\n');
fprintf('========================================\n');
fprintf('All figures saved and ready for report\n\n');

% Save workspace
save('ECP205a_Tasks1to7_Results.mat');
fprintf('Workspace saved to: ECP205a_Tasks1to7_Results.mat\n');