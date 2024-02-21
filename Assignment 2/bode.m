%% 
directory = '4a_1/CSV/';

% Get the list of files in the directory
file_list = dir(directory);

freq = [];
servo_amp = [];
error_amp = [];
phase_list = [];
gain = [];
% Iterate through the file list
for i = 1:length(file_list)
    % Display the name of each file
    % disp(file_list(i).name);
    if length(file_list(i).name) < 12
        continue;
    end
    fre = str2double(file_list(i).name(6:8));
    freq = [freq; fre];

    % data = csvread('CSV/Curve0.1Hz_0.csv');
    data = readmatrix(strcat(directory, file_list(i).name));
    data_new = data(3:end, :);
    s_amplitude = max(data_new(:, 3)) - min(data_new(:, 3));
    e_amplitude = max(data_new(:, 2)) - min(data_new(:, 2));
    servo_signal = data_new(:, 3);
    error_signal = data_new(:, 2);
    servo_amp = [servo_amp; s_amplitude];
    error_amp = [error_amp; e_amplitude];

    track_signal = servo_signal + error_signal;

    % Compute cross-correlation
    [R, lag] = xcorr(track_signal,servo_signal);
    
    % Find the lag with maximum cross-correlation
    [~, max_index] = max(abs(R));
    optimal_lag = lag(max_index);
    
    % Estimate phase difference (in radians)
    phase_difference = 2 * pi * optimal_lag / length(track_signal);
    
    % Estimate amplitude ratio
    amplitude_ratio = max(abs(servo_signal)) / max(abs(track_signal));
    gain = [gain; amplitude_ratio];
    % Convert phase difference to degrees
    phase_difference_degrees = rad2deg(phase_difference);
    phase_list = [phase_list; phase_difference_degrees];
    
    % Display results
    % disp(['Estimated Phase Difference: ', num2str(phase_difference_degrees), ' degrees']);
    % disp(['Estimated Amplitude Ratio: ', num2str(amplitude_ratio)]);

end

%%
subplot(2,1,1);
plot(freq(:), gain(:));
title('Plot of gain');
xlabel('frequency (Hz)');
ylabel('gain');

subplot(2,1,2);
plot(freq(:), phase_list(:));
title('Plot of phase');
xlabel('frequency (Hz)');
ylabel('phase (degree)');

%%
figure;
data = readmatrix(strcat(directory, file_list(4).name));
plot(data(:, 1), data(:, 2))
hold on;
plot(data(:, 1), data(:, 3))
track_signal = servo_signal + error_signal;
servo_signal = data(:, 3);
error_signal = data(:, 2);
track_signal = servo_signal + error_signal;
plot(data(:, 1), track_signal)
title('frequency = 1.8')
error_signal = data(:, 2);

