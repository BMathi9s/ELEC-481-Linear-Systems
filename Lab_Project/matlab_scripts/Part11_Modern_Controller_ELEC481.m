%% ELEC 481 - FINAL State-Feedback & Observer Design Script
% Group: Kenza Tarek, Achal Patel, Mathias Desrochers
%
% This script designs EVERYTHING for Section 5 (Guidelines 9-12):
%   1. The State-Feedback Controller (K)
%   2. The State-Observer (L)
%   3. The Feedforward Gain (Nbar)
%   4. The matrices for the Simulink Observer Block (A_obs, B_obs...)
%
% --- RUN THIS SCRIPT ONCE ---
% All variables will be loaded into your workspace for Simulink.
%
clc;
clear;
close all;

%% ========================================================================
%  PHASE 1: SYSTEM DEFINITION (From Report Section 3)
%  ========================================================================
fprintf('PHASE 1: Defining System...\n');
% --- Your Measured System Parameters ---
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

% --- THIS IS THE *REAL* OUTPUT MATRIX ---
% We can measure theta1 and theta2 with the encoders.
% y = [theta1; theta2]
C = [1, 0, 0, 0;
     0, 0, 1, 0];

D = [0;
     0];

% Check Controllability & Observability
ctrb_rank = rank(ctrb(A, B));
obsv_rank = rank(obsv(A, C));
fprintf('Controllability Rank: %d (Must be 4)\n', ctrb_rank);
fprintf('Observability Rank: %d (Must be 4)\n', obsv_rank);
if ctrb_rank < 4 || obsv_rank < 4
    error('System is not controllable or observable. Check parameters.');
end

%% ========================================================================
%  PHASE 2: CONTROLLER DESIGN (Guideline 9)
%  ========================================================================
fprintf('PHASE 2: Designing Controller K...\n');
% --- Your Controller Specs (Same as before) ---
zeta = 0.69;      % Corresponds to ~5% Overshoot
sigma = 2.3;    % Corresponds to Ts = 4.6/sigma = 2 sec
wn = sigma / zeta; % Natural frequency
wd = wn * sqrt(1-zeta^2); % Damped natural frequency

p1 = -sigma + 1j*wd;
p2 = -sigma - 1j*wd;
p3 = -sigma * 10;
p4 = -sigma * 10 - 1; 

P_controller = [p1, p2, p3, p4];

% --- Calculate Controller Gain K ---
K = place(A, B, P_controller);
fprintf('Controller Gain K:\n');
disp(K);

%% ========================================================================
%  PHASE 3: OBSERVER DESIGN (Guideline 11)
%  ========================================================================
fprintf('PHASE 3: Designing Observer L...\n');
% --- Observer Poles (4x faster than controller) ---
observer_speed_factor = 4; 
P_observer = P_controller * observer_speed_factor;

% --- Calculate Observer Gain L ---
L = place(A', C', P_observer)';
fprintf('Observer Gain L:\n');
disp(L);

%% ========================================================================
%  PHASE 4: FEEDFORWARD GAIN (Guideline 10)
%  ========================================================================
fprintf('PHASE 4: Calculating Feedforward Gain Nbar...\n');
% This calculation is correct, but be careful: This Nbar is for
% the IDEAL system (Sim A). When using an observer (Sim B), the
% steady-state gain is *different*. For simplicity in your report,
% we will re-use this Nbar.
C_theta2 = C(2, :); % C_theta2 = [0 0 1 0]
D_theta2 = D(2);   % D_theta2 = 0
sys_cl = ss(A - B*K, B, C_theta2, D_theta2);
gain = dcgain(sys_cl);
Nbar = 1 / gain;
fprintf('Feedforward Gain Nbar:\n');
disp(Nbar);

%% ========================================================================
%  PHASE 5: (!!! NEW !!!) MATRICES FOR SIMULINK OBSERVER BLOCK
%  ========================================================================
% These are the special matrices for your "Observer" block in Simulink.
% The observer's state equation is:
% x_hat_dot = (A-LC)x_hat + [B, L] * [u; y]
%
fprintf('PHASE 5: Calculating Observer Matrices for Simulink...\n');

A_obs = A - L*C;

% The observer block has TWO inputs:
% 1. The control signal 'u' (1 signal)
% 2. The *real* measurement 'y' (2 signals, theta1 & theta2)
% So, B_obs combines the B and L matrices.
B_obs = [B, L]; % This will be [4x3]

% The observer's output is the full estimated state x_hat
C_obs = eye(4); % This is a [4x4] identity matrix

% The D matrix for the observer
% It has 4 outputs (C_obs) and 3 inputs (B_obs)
D_obs = zeros(4, 3);

fprintf('\n--- Design Complete ---');
fprintf('\nAll variables (A, B, C, D, K, L, Nbar, A_obs, B_obs, C_obs, D_obs) are in your workspace.\n');