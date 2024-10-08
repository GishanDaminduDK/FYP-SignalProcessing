% fileFullPath_1 = 'D:\FYP_AI\small_drone\drone_with_propelers_inclined\master_0000_data.bin';
fileFullPath_1 = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\Radar_Data\metal_plate_distance_3m\master_0000_data.bin';
%fileFullPath_2 ='D:\FYP_AI\small_drone\Normal_envionment\master_0000_data.bin';
frameIdx =129;             % Index of the frame you want to read
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 64;              % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade (adjust based on your setup)
% Constants (adjust based on your radar parameters)
fc = 77e9;                  % Radar operating frequency (77 GHz for mmWave radar)
c = 3e8;                    % Speed of light (m/s)
sweepBandwidth = 3.16e9;     % Bandwidth of the FMCW radar sweep (3.16 GHz)
chirpDuration = 40e-6;       % Chirp duration (40 microseconds)

% FFT parameters
Nfft_range = 320;           % Number of FFT points for range dimension
Nfft_doppler = 13;  
% Read binary file data
[adcData1Complex] = readBinFile(fileFullPath_1, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
%[adcData1Complex_2] = readBinFile(fileFullPath_2, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
% Subtract the data from the two environments

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

doppler_res = 1 / (numChirpPerLoop * chirpDuration);           % Doppler resolution (Hz)
max_doppler = doppler_res * (Nfft_doppler / 2);                % Maximum Doppler shift (Hz)
doppler_axis = linspace(-max_doppler, max_doppler, Nfft_doppler);

% Convert Doppler frequency to velocity (m/s)
velocity_axis = doppler_axis * (c / (2 * fc));               % Velocity axis using Doppler shift

% Plot the Range-Velocity map (absolute value of FFT)
figure;
imagesc(velocity_axis, range_axis, 20*log10(abs(doppler_fft))); % Convert to dB for better visualization
xlabel('Velocity (m/s)');
ylabel('Range (m)');
title('Range-Velocity Map');
colorbar;
axis xy; % Correct axis orientation
% Ensure doppler_fft is a 2D matrix
doppler_fft = squeeze(doppler_fft(:, :, 1)); % Extract the first chirp for visualization

% Prepare the magnitude data for 3D plotting (convert to dB scale)
magnitude_dB = 20 * log10(abs(doppler_fft)); % Convert FFT magnitude to dB

% Adjust the meshgrid to match the dimensions of magnitude_dB
[Velocity, Range] = meshgrid(velocity_axis, range_axis); % Ensure dimensions align with the data

% Plot the 3D surface
figure;
surf(Velocity, Range, magnitude_dB, 'EdgeColor', 'none'); % Create a surface plot without edges
xlabel('Velocity (m/s)');
ylabel('Range (m)');
zlabel('Magnitude (dB)');
title('3D Range-Velocity-Magnitude Plot');
colorbar; % Add colorbar to represent the magnitude scale
view(3);  % Set the view to 3D
axis tight;


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
