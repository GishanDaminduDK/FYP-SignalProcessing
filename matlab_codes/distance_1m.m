fileFullPath = 'E:\Data_Set\Phanton_4_forward_01\new_two\master_0000_data.bin';
%fileFullPath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\metal_plates\master_0000_data_4m\master_0000_data_4m.bin';
frameIdx = 129;             % Index of the frame you want to read
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 128;              % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade (adjust based on your setup)
% Constants (adjust based on your radar parameters)
fc = 77e9;                  % Radar operating frequency (77 GHz for mmWave radar)
c = 3e8;                    % Speed of light (m/s)
sweepBandwidth = 0.899451e9;     % Bandwidth of the FMCW radar sweep (3.16 GHz)
chirpDuration = 30e-6;       % Chirp duration (40 microseconds)
% FFT parameters
Nfft_range = 300;           % Number of FFT points for range dimension
Nfft_doppler = 97;  
% Read binary file data
[adcData1Complex] = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
antennaIdx = 1;             % Antenna number (1st antenna)

% Extract the chirp ADC matrix for all loops and the selected antenna
chirp_ADC_matrix = adcData1Complex(:, :, antennaIdx, :);

% Remove singleton dimensions
chirp_ADC_matrix = squeeze(chirp_ADC_matrix);

% Perform 2D FFT along both the range (ADC samples) and Doppler (chirps) dimensions
range_fft = fft(chirp_ADC_matrix, Nfft_range, 1);               % FFT across range (ADC samples)
doppler_fft = fftshift(fft(range_fft, Nfft_doppler, 2), 2);     % FFT across Doppler (chirps), with shift

% Ensure doppler_fft is a 2D matrix for plotting
doppler_fft = squeeze(doppler_fft(:, :, 1)); % Extract the first chirp for visualization

% Calculate the range and velocity axis values
range_res = c / (2 * sweepBandwidth);                          % Range resolution (meters)
max_range = range_res * (Nfft_range - 1);                      % Maximum measurable range
range_axis = linspace(0, max_range, Nfft_range);               % Range axis for plotting

doppler_res = 1 / (numChirpPerLoop * chirpDuration*100);           % Doppler resolution (Hz)
max_doppler = doppler_res * (Nfft_doppler / 2);                % Maximum Doppler shift (Hz)
doppler_axis = linspace(-max_doppler, max_doppler, Nfft_doppler);

% Convert Doppler frequency to velocity (m/s)
velocity_axis = doppler_axis * (c / (2 * fc));               % Velocity axis using Doppler shift

% % Plot the Range-Velocity map (absolute value of FFT)
figure;
imagesc(velocity_axis, range_axis, 20*log10(abs(doppler_fft))); % Convert to dB for better visualization
xlabel('Velocity (m/s)');
ylabel('Range (m)');
title('Range-Velocity Map');
colorbar;
axis xy; % Correct axis orientation
range_fft_only = fft(chirp_ADC_matrix, Nfft_range, 1); % FFT across range (ADC samples)

% Calculate magnitude in dB
range_fft_magnitude_dB = 20*log10(abs(range_fft_only(:, 1, 1))); % Use the first chirp and RX channel for plotting

% Plot the Range FFT (Magnitude vs Range) in dB
figure;
plot(range_axis, range_fft_magnitude_dB);
xlabel('Range (m)');
ylabel('Magnitude (dB)');
title('Range FFT (Magnitude vs Range) in dB');
grid on;

% Plot 2: Range FFT in dBFS
max_magnitude = max(abs(range_fft_only(:, 1, 1)));  % Maximum possible magnitude for normalization
range_fft_magnitude_dBFS = 20*log10(abs(range_fft_only(:, 1, 1)) / max_magnitude); % Convert to dBFS

% Plot the Range FFT (Magnitude vs Range) in dBFS
figure;
plot(range_axis, range_fft_magnitude_dBFS);
xlabel('Range (m)');
ylabel('Magnitude (dBFS)');
title('Range FFT (Magnitude vs Range) in dBFS');
grid on;
% Parameters
% Parameters
sampling_rate = 10000e3;  % 8000 ksps

% Extract the time-domain signal (first chirp and first RX channel)
mean_of_all_chirps = mean(chirp_ADC_matrix, [2, 3]);
disp(size(mean_of_all_chirps));  % Should return [numSamplePerChirp, 1]
disp(mean_of_all_chirps); 
% Define the file path where the complex values will be saved
filePath = 'mean_chirp_phantom_new_1.txt';

% Open the file in write mode
fileID = fopen(filePath, 'w');

% Write each complex value to the file line by line
for i = 1:length(mean_of_all_chirps)
    fprintf(fileID, '%f + %fi\n', real(mean_of_all_chirps(i)), imag(mean_of_all_chirps(i)));
end

% Close the file
fclose(fileID);

disp(['File written to: ', filePath]);
time_domain_signal = mean_of_all_chirps;  % First chirp, first RX channel

real_signal = real(time_domain_signal);  % Real part of the signal
imag_signal = imag(time_domain_signal);  % Imaginary part of the signal

% Calculate magnitude and phase
magnitude_signal = abs(time_domain_signal);  % Magnitude of the complex signal
phase_signal = angle(time_domain_signal);    % Phase of the complex signal

% Create a time axis based on the number of samples and sampling rate
time_axis = (0:numSamplePerChirp-1) / sampling_rate;  % Time in seconds

% Plot Real and Imaginary Parts of the Time-Domain Signal
figure;
subplot(3, 1, 1);  % Create a 3-row plot layout, first plot
plot(time_axis, real_signal, 'b', 'DisplayName', 'Real Part');  % Real part in blue
hold on;
plot(time_axis, imag_signal, 'r', 'DisplayName', 'Imaginary Part');  % Imaginary part in red
hold off;
xlabel('Time (s)');
ylabel('Amplitude');
title('Real and Imaginary Parts of Time-Domain Signal');
grid on;
legend show;

% Plot Magnitude vs Time
subplot(3, 1, 2);  % Second plot
plot(time_axis, magnitude_signal, 'g', 'DisplayName', 'Magnitude');  % Magnitude in green
xlabel('Time (s)');
ylabel('Magnitude');
title('Magnitude vs Time');
grid on;

% Plot Phase vs Time
subplot(3, 1, 3);  % Third plot
plot(time_axis, phase_signal, 'm', 'DisplayName', 'Phase');  % Phase in magenta
xlabel('Time (s)');
ylabel('Phase (radians)');
title('Phase vs Time');
grid on;


function [adcData1Complex] = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice)
    Expected_Num_SamplesPerFrame = numSamplePerChirp * numChirpPerLoop * numLoops * numRXPerDevice * 2;
    fp = fopen(fileFullPath, 'r');
    
    if fp == -1
        error('File could not be opened.');
    end
    
    % Move to the desired frame in the file
    fseek(fp, (frameIdx - 1) * Expected_Num_SamplesPerFrame * 2, 'bof');
    adcData1 = fread(fp, Expected_Num_SamplesPerFrame, 'uint16');
    fclose(fp);
    
    % Convert the 16-bit data to signed integers
    neg = logical(bitget(adcData1, 16));
    adcData1(neg) = adcData1(neg) - 2^16;
    
    % Combine the I and Q channels into complex values
    adcData1 = adcData1(1:2:end) + 1j * adcData1(2:2:end);
    
    % Reshape and permute the data
    adcData1Complex = reshape(adcData1, numRXPerDevice, numSamplePerChirp, numChirpPerLoop, numLoops);
    adcData1Complex = permute(adcData1Complex, [2 4 1 3]);
end

