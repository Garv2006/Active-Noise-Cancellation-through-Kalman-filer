% Active Noise Control: Sudden Path Change Tracking
clear; clc; close all;

iterations = 6000;
jump_iter = 3000; % The moment the physical environment changes
filter_length = 32;

% Original and Delayed Paths
w_true_initial = fir1(filter_length - 1, 0.5)';
w_true_delayed = [zeros(3, 1); w_true_initial(1:end-3)]; % Delayed by 3 samples

% Generate inputs
u = filter(fir1(15, [0.2 0.8]), 1, randn(iterations, 1)); 
r = 0.1 * randn(iterations, 1);

% Generate desired signal with the abrupt jump in the middle
d1 = filter(w_true_initial, 1, u);
d2 = filter(w_true_delayed, 1, u);
d = [d1(1:jump_iter-1); d2(jump_iter:end)] + r;

% Initialize Filters (Same as before)
m_LMS = 0.01; w_LMS = zeros(filter_length, 1);
lambda_RLS = 0.999; P_RLS = (1/0.01) * eye(filter_length); w_RLS = zeros(filter_length, 1);
lambda_Kalman = 0.99999; Q1 = (1 - lambda_Kalman) * eye(filter_length); Q2 = var(r); 
F = eye(filter_length); K_kalman_cov = eye(filter_length); x_kalman = zeros(filter_length, 1);

u_buffer = zeros(filter_length, 1);
e_LMS_hist = zeros(iterations, 1); e_RLS_hist = zeros(iterations, 1); e_Kalman_hist = zeros(iterations, 1);

for n = 1:iterations
    u_buffer = [u(n); u_buffer(1:end-1)];
    d_n = d(n);

    % Determine which true path we are comparing against for misalignment
    if n < jump_iter
        current_w_true = w_true_initial;
    else
        current_w_true = w_true_delayed;
    end

    % LMS
    y_LMS = w_LMS' * u_buffer;
    e_LMS = d_n - y_LMS;
    w_LMS = w_LMS + m_LMS * u_buffer * e_LMS;
    e_LMS_hist(n) = e_LMS;

    % RLS
    y_RLS = w_RLS' * u_buffer;
    a_RLS = d_n - y_RLS; 
    p_RLS = P_RLS * u_buffer;
    k_RLS = p_RLS / (lambda_RLS + u_buffer' * p_RLS); 
    w_RLS = w_RLS + k_RLS * a_RLS;
    P_RLS = (1/lambda_RLS) * (P_RLS - k_RLS * u_buffer' * P_RLS);
    e_RLS_hist(n) = a_RLS;

    % Kalman
    C_n = u_buffer';
    G_n = (F * K_kalman_cov * C_n') / (C_n * K_kalman_cov * C_n' + Q2);
    a_Kalman = d_n - C_n * x_kalman;
    x_kalman = F * x_kalman + G_n * a_Kalman;
    K_n = K_kalman_cov - G_n * C_n * K_kalman_cov;
    K_kalman_cov = F * K_n * F' + Q1;
    e_Kalman_hist(n) = a_Kalman;
end

% Plotting
window_size = 50;
smooth_LMS = 10*log10(movmean(e_LMS_hist.^2, window_size));
smooth_RLS = 10*log10(movmean(e_RLS_hist.^2, window_size));
smooth_Kalman = 10*log10(movmean(e_Kalman_hist.^2, window_size));

figure;
plot(smooth_LMS, 'DisplayName', 'LMS', 'LineWidth', 1.2); hold on;
plot(smooth_RLS, 'DisplayName', 'RLS', 'LineWidth', 1.2);
plot(smooth_Kalman, 'DisplayName', 'Kalman', 'LineWidth', 1.2);
title('Response to Sudden Path Change (n=3000)');
xlabel('Iterations'); ylabel('Noise Level (dB)');
legend; grid on; axis([2800 4500 -25 5]); % Zoom in on the jump
%% 5. Compute Tracking / Recovery Metrics
clc;

% Define the threshold for recovery after the shock
recovery_thresh = -15; % dB

% Isolate the smoothed signals after the jump at iteration 3000
post_jump_LMS = smooth_LMS(jump_iter:end);
post_jump_RLS = smooth_RLS(jump_iter:end);
post_jump_Kalman = smooth_Kalman(jump_iter:end);

% Find the number of iterations it takes to drop back below the threshold
rec_LMS = find(post_jump_LMS < recovery_thresh, 1);
if isempty(rec_LMS), rec_LMS = NaN; end

rec_RLS = find(post_jump_RLS < recovery_thresh, 1);
if isempty(rec_RLS), rec_RLS = NaN; end

rec_Kalman = find(post_jump_Kalman < recovery_thresh, 1);
if isempty(rec_Kalman), rec_Kalman = NaN; end

% --- Print the Results to Command Window ---
fprintf('====================================================\n');
fprintf('     RECOVERY METRICS (SUDDEN PATH CHANGE)          \n');
fprintf('====================================================\n\n');
fprintf('Iterations to recover to %d dB after the shock:\n', recovery_thresh);
fprintf('----------------------------------------------------\n');
fprintf('   LMS:    %7d iterations\n', rec_LMS);
fprintf('   RLS:    %7d iterations\n', rec_RLS);
fprintf('   Kalman: %7d iterations\n\n', rec_Kalman);