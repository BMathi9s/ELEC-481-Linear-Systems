%% ELEC 481 - State-Feedback Controller Design Script
% Group: Kenza Tarek, Achal Patel, Mathias Desrochers
%
% This script designs the state-feedback controller (K),
% the state-observer (L), and the feedforward gain (Nbar)
% for the 2-DOF Torsional Control System (ECP Model 205).
%
% --- RUN THIS SCRIPT FIRST ---
% The results (K, L, Nbar, etc.) are used in the Simulink model.
%
clc;
clear;
close all;

%% ========================================================================
%  PHASE 1: SYSTEM DEFINITION (From Report Section 3)
%  ========================================================================
% --- USER INPUT: Enter your measured system parameters ---
% These values come from your system identification (Lab Manual 6.1)
% *** REPLACE THESE PLACEHOLDERS WITH YOUR VALUES ***
J1 = 0.0108;    % kg-m^2 (Inertia of Disk 1 + motor)
J2 = 0.0103;    % kg-m^2 (Inertia of Disk 2)
k1 = 1.37;      % N-m/rad (Spring constant between J1 and J2)
k2 = 0;         % N-m/rad (Spring constant of J2 to frame, 0 for free-clamped)
c1 = 0.007;     % N-m-s/rad (Damping coefficient at J1)
c2 = 0.001;     % N-m-s/rad (Damping coefficient at J2)

% --- State-Space Matrices (from manual-2dofsection.pdf, Eq 5.1-3) ---
% State vector x = [theta1; theta1_dot; theta2; theta2_dot]
A = [0,             1,            0,                0;
     -k1/J1,        -c1/J1,       k1/J1,            0;
     0,             0,            0,                1;
     k1/J2,         0,            -(k1+k2)/J2,      -c2/J2];

B = [0;
     1/J1;
     0;
     0];

% Output vector y = [theta1; theta2]
% We can measure both positions with the encoders.
C = [1, 0, 0, 0;
     0, 0, 1, 0];

D = [0;
     0];
disp("A")
disp(A)
disp("B")
disp(B)
disp("C")
disp(C)
disp("D")
disp(D)

% --- System Analysis (Good to check!) ---
% Check Controllability (Must be 4, or full rank)
ctrb_rank = rank(ctrb(A, B));
fprintf('Controllability Rank: %d (Must be 4)\n', ctrb_rank);
if ctrb_rank < 4
    error('System is not controllable. Check parameters.');
end

% Check Observability (Must be 4, or full rank)
obsv_rank = rank(obsv(A, C));
fprintf('Observability Rank: %d (Must be 4)\n', obsv_rank);
if obsv_rank < 4
    error('System is not observable. Check C matrix or parameters.');
end

fprintf('\n--- System Model Defined ---\n');

%% ========================================================================
%  PHASE 2: CONTROLLER DESIGN (Guideline 9)
%  ========================================================================
% Goal: Find gain K for u = -Kx.
% We use pole placement to meet the specs from Section 4.

% --- USER INPUT: Define your controller poles ---
% These must match the specs you chose for your PID controller (Guideline 6)
% Example: 5% Overshoot (zeta = 0.69) & 2 sec Settling Time (sigma = 2.3)
%
% 1. Dominant poles: p1,p2 = -sigma +/- j*wd
zeta = 0.69;      % Corresponds to ~5% Overshoot
sigma = 2.3;    % Corresponds to Ts = 4.6/sigma = 2 sec
wn = sigma / zeta; % Natural frequency
wd = wn * sqrt(1-zeta^2); % Damped natural frequency

p1 = -sigma + 1j*wd;
p2 = -sigma - 1j*wd;

% 2. Fast poles: Place these 5-10x faster (further left) than dominant poles
%    so they don't affect the step response.
p3 = -sigma * 10;
p4 = -sigma * 10 - 1; % Just to make them distinct

% Final set of 4 desired closed-loop poles
P_controller = [p1, p2, p3, p4];

% --- Calculate Controller Gain K ---
% This is the "answer" for Guideline 9.
% The 'place' function solves Ackermann's formula.
K = place(A, B, P_controller);

fprintf('Controller Gain K:\n');
disp(K);

%% ========================================================================
%  PHASE 3: OBSERVER DESIGN (Guideline 11)
%  ========================================================================
% Goal: Find observer gain L to estimate x_hat from measurements y.
%       The observer equation is: x_hat_dot = (A-LC)x_hat + Bu + Ly
%
% Rule: Observer poles must be 2-5x faster than controller poles
%       so the estimate (x_hat) converges before the controller (K) reacts.

% --- USER INPUT: Define your observer poles ---
observer_speed_factor = 4; % Make them 4x faster
P_observer = P_controller * observer_speed_factor;

% --- Calculate Observer Gain L ---
% We use 'place' on the (A', C') system.
% This is the "answer" for Guideline 11.
L = place(A', C', P_observer)';

fprintf('Observer Gain L:\n');
disp(L);

%% ========================================================================
%  PHASE 4: FEEDFORWARD GAIN (Guideline 10)
%  ========================================================================
% Goal: Find Nbar so that our system u = -Kx + Nbar*r
%       has a steady-state output of 1 for a step input r=1.
%
% We want to control the output theta2 (Disk 2), which is row 2 of our
% output C matrix (C_theta2 = C(2, :)).
C_theta2 = C(2, :); % C_theta2 = [0 0 1 0]
D_theta2 = D(2);   % D_theta2 = 0

% Calculate the steady-state gain of the K-controlled system
sys_cl = ss(A - B*K, B, C_theta2, D_theta2);
gain = dcgain(sys_cl);

% Nbar is the inverse of that gain.
Nbar = 1 / gain;

fprintf('Feedforward Gain Nbar:\n');
disp(Nbar);

%% ========================================================================
%  PHASE 5: SIMULINK MATRICES (Guideline 11)
%  ========================================================================
% These are the final matrices you will use in your Simulink
% "Observer" block. This makes setup easy.

A_obs = A - L*C;
B_obs = [B, L]; % Observer has TWO inputs: u (from B) and y (from L)
C_obs = eye(size(A)); % We want to output the full estimated state x_hat
D_obs = [zeros(size(B)), zeros(size(L))];


%fopr simulink we had to set the C and D to zero so that we can multiply by
%the gain, but then the spate model block was output all 4 so we used a selector to 
%fix it 
C_system = eye(4);
D_system = zeros(4,1);


fprintf('\n--- Design Complete ---');
fprintf('\nVariables K, L, Nbar, A_obs, B_obs, C_obs, D_obs are in your workspace.\n');
fprintf('Use these values in your Simulink model.\n');