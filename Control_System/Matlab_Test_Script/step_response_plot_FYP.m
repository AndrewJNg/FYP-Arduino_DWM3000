
fclose all; close all; clear all; clc;% Load data with original column names
% data = readtable('encoder_data9.csv', 'VariableNamingRule', 'preserve');
% data = readtable('TT_Speed_stepping.csv', 'VariableNamingRule', 'preserve');
% data = readtable('TT_Speed_stepping_AS56002.csv', 'VariableNamingRule', 'preserve');
% data = readtable('TT_Speed_stepping_AS5600_step.csv', 'VariableNamingRule', 'preserve');
% data = readtable('TT_Speed_stepping_AS5600_path.csv', 'VariableNamingRule', 'preserve');
% data = readtable('TT_Speed_stepping_AS5600_pid.csv', 'VariableNamingRule', 'preserve');
data = readtable('TT_Speed_stepping_AS5600_1m.csv', 'VariableNamingRule', 'preserve');

% data = readtable('step_half.csv', 'VariableNamingRule', 'preserve');
% data = readtable('step_half08.csv', 'VariableNamingRule', 'preserve');

%%
% Display the variable names to confirm column headers
disp(data.Properties.VariableNames);

% Extract variables using exact names
time = data.("time");
leftPosition = data.("leftEncoderCount");
rightPosition = data.("rightEncoderCount");
%%
% Convert time to seconds and calculate velocity
time = time / 1000; 
time_diff = diff(time);
leftVelocity = diff(leftPosition) ./ time_diff;
rightVelocity = diff(rightPosition) ./ time_diff;

% Adjust time to match size of velocity data
time = time(1:end-1);

% Find the first point where velocity starts increasing for left and right motors
left_start_idx = find(leftVelocity > 0, 1, 'first');
right_start_idx = find(rightVelocity > 0, 1, 'first');
start_idx = min(left_start_idx, right_start_idx);  % Start from the earliest increase

% Crop data starting from the calculated index
time = time(start_idx:end);
leftPosition = leftPosition(start_idx:end-1);
rightPosition = rightPosition(start_idx:end-1);
leftVelocity = leftVelocity(start_idx:end);
rightVelocity = rightVelocity(start_idx:end);

% Reset time to start from 0
time = time - time(1);

% Apply moving average filter (window size = 5 samples)
windowSize = 100;
filteredLeftVelocity = movmean(leftVelocity, windowSize);
filteredRightVelocity = movmean(rightVelocity, windowSize);


% Design a low-pass Butterworth filter with a cutoff frequency of 5 Hz
Fs = 1 / mean(time_diff);  % Sampling frequency
cutoff_freq = 5;  % Cutoff frequency in Hz
[b, a] = butter(4, cutoff_freq / (Fs / 2), 'low');  % 4th-order low-pass filter

% Apply the filter to the original velocity data
low_passedLeftVelocity = filtfilt(b, a, leftVelocity);
low_passedRightVelocity = filtfilt(b, a, rightVelocity);



%%
% Plot positions, deltas, and velocities
figure;

subplot(4, 1, 1);
hold on;
plot(time, leftPosition, 'b-', 'DisplayName', 'Left Motor Position');
plot(time, rightPosition, 'r-', 'DisplayName', 'Right Motor Position');
xlabel('Time (s)');
ylabel('Position (mm)');
title('Motor Positions Over Time');
legend;
grid on;
hold off;


subplot(4, 1, 2);
hold on;
plot(time, leftVelocity, 'b-', 'DisplayName', 'Left Motor Velocity (Original)');
plot(time, rightVelocity, 'r-', 'DisplayName', 'Right Motor Velocity (Original)');
plot(time, filteredLeftVelocity, 'c-', 'LineWidth', 1.5, 'DisplayName', 'Left Motor Velocity (Filtered)');
plot(time, filteredRightVelocity, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Right Motor Velocity (Filtered)');
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
title('Motor Velocities Over Time');
legend;
grid on;
hold off;


%%

% Plot low-pass filtered velocity
subplot(4, 1, 3);
hold on;
plot(time, leftVelocity, 'b-', 'DisplayName', 'Left Motor Velocity (Original)');
plot(time, rightVelocity, 'r-', 'DisplayName', 'Right Motor Velocity (Original)');
plot(time, low_passedLeftVelocity, 'b-', 'LineWidth', 2, 'DisplayName', 'Left Motor Velocity (Low-Pass 5 Hz)');
plot(time, low_passedRightVelocity, 'r-', 'LineWidth', 2, 'DisplayName', 'Right Motor Velocity (Low-Pass 5 Hz)');
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
title('Low-Pass Filtered Motor Velocities (5 Hz Cutoff)');
legend;
% xlim([0,0.2])
grid on;
hold off;
%%
% Calculate frequency response for left and right motor velocities
Fs = 1 / mean(time_diff);  % Sampling frequency
n = length(leftVelocity); % Number of samples

% Frequency axis centered around zero
f = (-n/2:n/2-1) * (Fs/n);

% FFT of the filtered velocity signals and shift zero frequency component to center
leftVelocity_fft = fftshift(abs(fft(leftVelocity)));
rightVelocity_fft = fftshift(abs(fft(rightVelocity)));

% Plot frequency response of the motor velocities using scatter plot
subplot(4, 1, 4);
hold on;
scatter(f, leftVelocity_fft, 'b', 'DisplayName', 'Left Motor Velocity Frequency Response');
scatter(f, rightVelocity_fft, 'r', 'DisplayName', 'Right Motor Velocity Frequency Response');
xlabel('Frequency (Hz)');
ylabel('Amplitude');
title('Frequency Response of Motor Velocity');
xlim([-2 2]); % Center plot between -10 and 10 Hz
legend;
grid on;
hold off;