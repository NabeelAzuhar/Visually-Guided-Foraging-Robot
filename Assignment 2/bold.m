%% 
directory = 'CSV_4/';
directory2 = 'CSV_pid_4/';
% directory = 'CSV_4/';
% Get the list of files in the directory
file_list = dir(directory);

freq = [];
servo_amp = [];
error_amp = [];
phase_list = [];
phase_list_perf = [];
gain = [];
gain_perf = [];
max_angle_list = [];
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
    time = data_new(:, 1);
    servo_signal = data_new(:, 3);
    error_signal = data_new(:, 2);
    servo_amp = [servo_amp; s_amplitude];
    error_amp = [error_amp; e_amplitude];

    track_signal = servo_signal + error_signal;

    % perfect tracking angle
    max_angle = rad2deg(atan(15/11.5)); % before PID
%     max_angle = rad2deg(atan(16/34)); % before PID
    max_angle_list = [max_angle_list; max_angle];
    perf_signal = max_angle * cos(2 * pi * fre * time);

    % Compute cross-correlation
    [R, lag] = xcorr(track_signal, servo_signal);
    [R_perf, lag_perf] = xcorr(servo_signal, perf_signal);
    
    % Find the lag with maximum cross-correlation
    [~, max_index] = max(abs(R));
    optimal_lag = lag(max_index);
    [~, max_index_perf] = max(abs(R_perf));
    optimal_lag_perf = lag(max_index_perf);
    
    % Estimate phase difference (in radians)
    phase_difference = 2 * pi * optimal_lag / length(track_signal);
    phase_difference_perf = 2 * pi * optimal_lag_perf / length(perf_signal);
    
    % Estimate amplitude ratio
    RMS_amplitude_servo = sqrt(mean(servo_signal.^2));
    RMS_amplitude_track = sqrt(mean(track_signal.^2));
    RMS_amplitude_perf = sqrt(mean(perf_signal.^2));

    gain_db = 20 * log10(RMS_amplitude_servo / RMS_amplitude_track);
    gain_db_perf = 20 * log10(RMS_amplitude_servo / RMS_amplitude_perf);
    gain = [gain; gain_db];
    gain_perf = [gain_perf; gain_db_perf];
    % Convert phase difference to degrees
    phase_difference_degrees = rad2deg(phase_difference);
    phase_list = [phase_list; phase_difference_degrees];
    phase_difference_degrees_perf = rad2deg(phase_difference_perf);
    phase_list_perf = [phase_list_perf; phase_difference_degrees_perf];
    
    % Display results
    % disp(['Estimated Phase Difference: ', num2str(phase_difference_degrees), ' degrees']);
    % disp(['Estimated Amplitude Ratio: ', num2str(amplitude_ratio)]);

end

%%
file_list = dir(directory2);

freq_pid = [];
servo_amp_pid = [];
error_amp_pid = [];
phase_list_pid = [];
phase_list_perf_pid = [];
gain_pid = [];
gain_perf_pid = [];
max_angle_list_pid = [];
% Iterate through the file list
for i = 1:length(file_list)
    % Display the name of each file
    % disp(file_list(i).name);
    if length(file_list(i).name) < 12
        continue;
    end
    fre = str2double(file_list(i).name(6:8));
    freq_pid = [freq_pid; fre];

    % data = csvread('CSV/Curve0.1Hz_0.csv');
    data = readmatrix(strcat(directory2, file_list(i).name));
    data_new = data(3:end, :);
    s_amplitude = max(data_new(:, 3)) - min(data_new(:, 3));
    e_amplitude = max(data_new(:, 2)) - min(data_new(:, 2));
    time = data_new(:, 1);
    servo_signal = data_new(:, 3);
    error_signal = data_new(:, 2);
    servo_amp_pid = [servo_amp_pid; s_amplitude];
    error_amp_pid = [error_amp_pid; e_amplitude];

    track_signal = servo_signal + error_signal;

    % perfect tracking angle
    max_angle = rad2deg(atan(16/34)); % after PID
    max_angle_list_pid = [max_angle_list_pid; max_angle];
    perf_signal = max_angle * cos(2 * pi * fre * time);

    % Compute cross-correlation
    [R, lag] = xcorr(track_signal, servo_signal);
    [R_perf, lag_perf] = xcorr(servo_signal, perf_signal);
    
    % Find the lag with maximum cross-correlation
    [~, max_index] = max(abs(R));
    optimal_lag = lag(max_index);
    [~, max_index_perf] = max(abs(R_perf));
    optimal_lag_perf = lag(max_index_perf);
    
    % Estimate phase difference (in radians)
    phase_difference = 2 * pi * optimal_lag / length(track_signal);
    phase_difference_perf = 2 * pi * optimal_lag_perf / length(perf_signal);
    
    % Estimate amplitude ratio
    RMS_amplitude_servo = sqrt(mean(servo_signal.^2));
    RMS_amplitude_track = sqrt(mean(track_signal.^2));
    RMS_amplitude_perf = sqrt(mean(perf_signal.^2));

    gain_db = 20 * log10(RMS_amplitude_servo / RMS_amplitude_track);
    gain_db_perf = 20 * log10(RMS_amplitude_servo / RMS_amplitude_perf);
    gain_pid = [gain_pid; gain_db];
    gain_perf_pid = [gain_perf_pid; gain_db_perf];
    % Convert phase difference to degrees
    phase_difference_degrees = rad2deg(phase_difference);
    phase_list_pid = [phase_list_pid; phase_difference_degrees];
    phase_difference_degrees_perf = rad2deg(phase_difference_perf);
    phase_list_perf_pid = [phase_list_perf_pid; phase_difference_degrees_perf];
    
    % Display results
    % disp(['Estimated Phase Difference: ', num2str(phase_difference_degrees), ' degrees']);
    % disp(['Estimated Amplitude Ratio: ', num2str(amplitude_ratio)]);

end

%%
% All
% figure;
% subplot(2,1,1);
% plot(freq, gain_perf); % (1: 7)
% title('Plot of gain');
% xlabel('frequency (Hz)');
% ylabel('gain (dB)');
% 
% subplot(2,1,2);
% plot(freq, phase_list_perf);
% title('Plot of phase');
% xlabel('frequency (Hz)');
% ylabel('phase (degree)');

% Till 0.5 hz
% figure;
% subplot(2,1,1);
% plot(freq(1:5), gain_perf((1:5))); % (1: 7)
% title('Plot of gain');
% xlabel('frequency (Hz)');
% ylabel('gain (dB)');
% 
% subplot(2,1,2);
% plot(freq(1:5), phase_list_perf(1:5));
% title('Plot of phase');
% xlabel('frequency (Hz)');
% ylabel('phase (degree)');


% All 3:
figure;
subplot(2, 2, 1)
plot(freq, gain_perf);
title('Gain plot before PID tuning');
xlabel('frequency (Hz)');
ylabel('gain (dB)');

subplot(2, 2, 2)
plot(freq, phase_list_perf);
title('Phase plot before PID tuning');
xlabel('frequency (Hz)');
ylabel('phase (degree)');

subplot(2, 2, 3)
plot(freq_pid(1:5), gain_pid(1:5));
title('Gain plot after PID tuning');
xlabel('frequency (Hz)');
ylabel('gain (dB)');

subplot(2, 2, 4)
plot(freq_pid(1:5), phase_list_pid(1:5));
title('Phase plot after PID tuning');
xlabel('frequency (Hz)');
ylabel('phase (degree)');





% %% 
% data = readmatrix(strcat('CSV/', file_list(11).name));
% data_new = data(3:end, :);
% s_amplitude = max(data_new(:, 3)) - min(data_new(:, 3));
% e_amplitude = max(data_new(:, 2)) - min(data_new(:, 2));
% servo_signal = data_new(:, 3);
% error_signal = data_new(:, 2);
% track_signal = servo_signal + error_signal;
% figure;
% hold on;
% plot(data_new(:, 1), servo_signal);
% plot(data_new(:, 1), error_signal);
% plot(data_new(:, 1), track_signal);
% legend('servo', 'error', 'target');
% 
% % Compute cross-correlation
%     [R, lag] = xcorr(track_signal, servo_signal);
%     
%     % Find the lag with maximum cross-correlation
%     [~, max_index] = max(abs(R));
%     optimal_lag = lag(max_index);
%     
%     % Estimate phase difference (in radians)
%     phase_difference = 2 * pi * optimal_lag / length(track_signal);
