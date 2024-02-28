clear all
clc
%% 
directory = 'CSV_pid_4/';

% Get the list of files in the directory
file_list = dir(directory);

freq = [];
p2p_servo_amp = [];
p2p_error_amp = [];
rms_servo_amp = [];
rms_error_amp = [];
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

    % data = csvread('CSV/Curve0.1Hz_0.csv');
    data = readmatrix(strcat(directory, file_list(i).name));
    data_new = data(3:end, :);
   
    time = data_new(:, 1);
    servo_signal = data_new(:, 3);
    error_signal = data_new(:, 2);
    
    if length(time) < 9
        continue;
    end
    
    freq = [freq; fre];

    % Peak to peak amplitude
    p2p_s_amp = max(servo_signal) - min(servo_signal);
    p2p_e_amp = max(error_signal) - min(error_signal);
    p2p_servo_amp = [p2p_servo_amp; p2p_s_amp];
    p2p_error_amp = [p2p_error_amp; p2p_e_amp];

    % RMS amplitude
    rms_servo_amp = [rms_servo_amp; rms(servo_signal)];
    rms_error_amp = [rms_error_amp; rms(error_signal)];

    % Gain
    track_signal = servo_signal + error_signal;
    gain = [gain; rms(servo_signal) / rms(track_signal)];

    % Phase difference
    % Calculate the cross-spectral density
    [csd, frequency] = cpsd(track_signal, servo_signal, [], [], [], 1/(time(2)-time(1)));
    
    % Find the phase difference
    phase_difference = mean(angle(csd));
    
    % Convert phase difference to degrees
    phase_difference_degrees = rad2deg(phase_difference);
    phase_list = [phase_list; phase_difference_degrees];
    
        
    
    
    
    
%     % Compute cross-correlation
%     [R, lag] = xcorr(track_signal,servo_signal);
%     
%     % Find the lag with maximum cross-correlation
%     [~, max_index] = max(abs(R));
%     optimal_lag = lag(max_index);
%     
%     % Estimate phase difference (in radians)
%     phase_difference = 2 * pi * optimal_lag / length(track_signal);
%     
%     % Estimate amplitude ratio
%     amplitude_ratio = max(abs(servo_signal)) / max(abs(track_signal));
%     gain = [gain; amplitude_ratio];
%     % Convert phase difference to degrees
%     phase_difference_degrees = rad2deg(phase_difference);
%     phase_list = [phase_list; phase_difference_degrees];
%     
    % Display results
    % disp(['Estimated Phase Difference: ', num2str(phase_difference_degrees), ' degrees']);
    % disp(['Estimated Amplitude Ratio: ', num2str(amplitude_ratio)]);

end

%% Plotting

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
servo_signal = data(:, 3);
error_signal = data(:, 2);
track_signal = servo_signal + error_signal;
plot(data(:, 1), track_signal)
title('frequency = 1.8')
error_signal = data(:, 2);

