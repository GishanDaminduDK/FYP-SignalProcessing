fileFullPath_1 = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\Radar_Data\waruna_drone\waruna_forward\master_0000_data.bin';
%fileFullPath_2 ='D:\Drone-Swarm-Detection-with-AWR2243\Our data\Radar_Data\waruna_drone\normal_envi\master_0000_data.bin';
outputFilePath = 'chirp_samples_fly_drone_15.txt'; % Output text file
file_paths = {fileFullPath_1} % Use cell array for paths
frame_indices = [80,40,40,60]; % Use square brackets for array
%fileFullPath_2 ='D:\FYP_AI\small_drone\Normal_envionment\master_0000_data.bin';
frameIdx = 15;             % Index of the frame you want to read
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
antennaIdx = 4; 
% FFT parameters
Nfft_range = 300;           % Number of FFT points for range dimension
Nfft_doppler = 97;  
% Read binary file data
%[adcData1Complex_2] = readBinFile(fileFullPath_2, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
% Subtract the data from the two environments
adc_raw_data = 0; % Initialize adc_raw_data as zero matrix with the same dimensions as adcData

% Read binary file data
for i = 1:length(file_paths)
    [adcData] = readBinFile(file_paths{i}, frame_indices(i), numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
    
    % Initialize adc_raw_data on the first iteration
    if i == 1
        adc_raw_data = zeros(size(adcData)); % Initialize to match the size of adcData
    end
    
    adc_raw_data = adc_raw_data+adcData; % Add matrices element-wise
end

% Extract the chirp ADC matrix for all loops and the selected antenna
antennaIdx = 4;
chirp_ADC_matrix = adc_raw_data(:, :, antennaIdx, :);
% Open the output text file for writing
fid = fopen(outputFilePath, 'w');
if fid == -1
    error('Failed to open the output file.');
end

% Write chirp samples to the file, loop by loop, chirp by chirp
for loopIdx = 1:numLoops
    for chirpIdx = 1:numChirpPerLoop
        samples = chirp_ADC_matrix(:, loopIdx, chirpIdx);
        
        % Write each complex sample on a new line in the format: real + j*imag
        for sampleIdx = 1:numSamplePerChirp
            realPart = real(samples(sampleIdx));
            imagPart = imag(samples(sampleIdx));
            fprintf(fid, '%f + j*%f\n', realPart, imagPart);
        end
    end
end

% Close the file
fclose(fid);

%disp(['Chirp samples have been written to ', outputFilePath]);
% Remove singleton dimensions and reshape for FFT processing
chirp_ADC_matrix = squeeze(chirp_ADC_matrix);
reshaped_matrix = reshape(chirp_ADC_matrix, numSamplePerChirp, numLoops * numChirpPerLoop);
disp(size(reshaped_matrix));  % Should display [256, numLoops*numChirpPerLoop]
radar_data = reshaped_matrix';
disp(size(radar_data));


% Perform Range FFT on the samples
% Assuming radar_data, chirp_duration, slope, and range_shift are already defined
slope = 29.9817e12;
chirp_duration = 30e-6;

% Calculate the number of samples in radar_data
n_samples = size(radar_data, 2);

% Calculate the sample rate based on the number of samples and chirp duration
sample_rate = n_samples / chirp_duration;

% Create the time vector
time = (0:n_samples-1) / sample_rate;
range_shift = 0;

% Calculate the frequency shift due to range shift
frequency_shift = 2 * slope * range_shift / (3e8); % Speed of light in m/s

% Calculate the phase shift
phase_shift = 2 * pi * frequency_shift * time;

% Apply the phase shift to the radar data
shifted_data_array = radar_data .* exp(-1j * phase_shift);
velocity_shift=0;

n_chirps = size(shifted_data_array, 1);  % Get the number of chirps
time = (0:n_chirps-1)' * chirp_duration;  % Create the time vector as a column vector

% Calculate the frequency shift due to velocity shift
frequency_shift = 2 * slope * velocity_shift / (3e8);  % Speed of light in m/s

% Calculate the phase shift
phase_shift = 2 * pi * frequency_shift * time;  % Phase shift calculation

% Apply the phase shift to the radar data
shifted_data_array = shifted_data_array .* exp(1j * phase_shift);  % Element-wise multiplication

% Perform Range FFT along the first dimension
range_fft = fft(shifted_data_array', Nfft_range, 1);

% Perform Doppler FFT along the second dimension and apply fftshift
doppler_fft = fftshift(fft(range_fft, Nfft_doppler, 2), 2);
% Display the size of doppler_fft for verification
disp(size(doppler_fft));  % Should display [Nfft_range, Nfft_doppler]
% Extract the first chirp if required for visualization or other processing
doppler_fft = squeeze(doppler_fft(:, :, 1));

% Calculate axes
range_res = c / (2 * sweepBandwidth);
max_range = range_res * (Nfft_range - 1);
range_axis = linspace(0, max_range, Nfft_range);

lambda = c / fc;
maximum_velocity = lambda / (chirpDuration * 4);
velocity_axis = linspace(-maximum_velocity, maximum_velocity, Nfft_doppler);

velocity_Resolution  = lambda / (2 * numLoops * chirpDuration * 12);
% CFAR processing
doppler_fft = doppler_fft * velocity_Resolution;              % Velocity axis using Doppler shift
magnitude_array = 20*log10(abs(doppler_fft));
% Create a logical array that checks if values in columns 48, 49, or 50 exceed 110
%Find the rows where values in columns 48, 49, or 50 are less than 110
% Find the rows where values in columns 48, 49, or 50 exceed 110
% magnitude_array(:, 48)=20;
magnitude_array(:, 49)=40;
% magnitude_array(:, 50)=20;
%magnitude_array(1:10, :) = 40;    % Set row 100 to 40
% Display the row numbers and corresponding values in columns 48, 49, and 50
% for i = 1:length(rows_exceeding_110)
%     row = rows_exceeding_110(i);
%     fprintf('Row: %d, Col 48: %.2f, Col 49: %.2f, Col 50: %.2f\n', row, ...
%         magnitude_array(row, 48), magnitude_array(row, 49), magnitude_array(row, 50));
% end

%rows_to_replace = magnitude_array(:, 48) < 110 | magnitude_array(:, 49) < 110 | magnitude_array(:, 50) < 110;

% Replace the values in those rows in columns 48, 49, and 50 with 80
%magnitude_array(rows_to_replace, 48:50) = 70;
% Find the maximum value in magnitude_array and its corresponding indices
% Find the maximum value and its location in the array
[max_value, linear_index] = max(magnitude_array(:));
[row, col] = ind2sub(size(magnitude_array), linear_index);
% Display the maximum value and its row and column indices
fprintf('Maximum value: %.2f found at Row: %d, Column: %d\n', max_value, row, col);
% Set all values to 30 initially
magnitude_array(:) = 10;
magnitude_array(row, col) = 72.3;
% % Define the size of the neighborhood to retain around the maximum value
% % (e.g., a 3x3 neighborhood centered on the maximum value)
% neighborhood_size = 1; % Adjust this for a larger neighborhood
% 
% % Calculate the row and column indices for the neighborhood
% row_start = max(1, row - neighborhood_size);
% row_end = min(size(magnitude_array, 1), row + neighborhood_size);
% col_start = max(1, col - neighborhood_size);
% col_end = min(size(magnitude_array, 2), col + neighborhood_size);
% 
% % Keep the original values in the specified neighborhood
% magnitude_array(row_start:row_end, col_start:col_end) = ...
%     magnitude_array(row_start:row_end, col_start:col_end) .* (magnitude_array(row_start:row_end, col_start:col_end) == max_value) ...
%     + magnitude_array(row_start:row_end, col_start:col_end);

% Display the result
fprintf('All values set to 30, except the maximum value and its neighboring area.\n');


% Display the row numbers
%disp(rows_exceeding_110);
figure;
imagesc(velocity_axis, range_axis,magnitude_array); % Convert to dB for better visualization
xlabel('Velocity (m/s)');
ylabel('Range (m)');
title('Range-Velocity Map');
colorbar;
axis xy; % Correct axis orientation
% Function to read the binary radar data file
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
