
fileFullPath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\metal_plates\master_0000_data_1m\master_0000_data_1m.bin';
%fileFullPath_2 ='D:\FYP_AI\small_drone\Normal_envionment\master_0000_data.bin';
frameIdx = 49;             % Index of the frame you want to read
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops =128;              % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade (adjust based on your setup)
% Constants (adjust based on your radar parameters)
fc = 77e9;                  % Radar operating frequency (77 GHz for mmWave radar)
c = 3e8;                    % Speed of light (m/s)
sweepBandwidth = 0.7675e9;     % Bandwidth of the FMCW radar sweep (3.16 GHz)
chirpDuration = 40e-6;       % Chirp duration (40 microseconds)

% FFT parameters
Nfft_range = 320;           % Number of FFT points for range dimension
Nfft_doppler = 13;  
% STFT-based Spectrogram with Chirp Mean
% Constants (from your provided setup)
fc = 77e9;                  % Radar operating frequency (77 GHz for mmWave radar)
c = 3e8;                    % Speed of light (m/s)

samplingRate = 10000e3;       % Sampling rate in Hz (8000 ksps)
% Read the radar data (your file reading function)
[adcData1Complex] = readBinFile(fileFullPath_1, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);

antennaIdx = 1;
% Extract the chirp ADC matrix for all loops and the selected antenna
chirp_ADC_matrix = adcData1Complex(:, :, antennaIdx, :);

% Remove singleton dimensions
chirp_ADC_matrix = squeeze(chirp_ADC_matrix);
concatenated_signal = [];  

for chirpIdx = 1:64  % Loop through all 64 chirp loops
    for individual_chirp = 1:12  % Loop through all 12 chirps in each loop
        % Extract the signal for the current chirp (across all samples)
        signal = squeeze(adcData1Complex(:, chirpIdx, antennaIdx, individual_chirp)); % Select a specific antenna and loop
          % Display the signal (optional, for debugging)
        
        % Concatenate the signal
        concatenated_signal = [concatenated_signal; signal];  % Vertically concatenate the signal into concatenated_signal
    end
end
disp(length(concatenated_signal))
disp(size(concatenated_signal))

% Define the file path where you want to save the text file
outputFilePath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\concatenated_signal_data.txt';

% Open the file for writing (this will create the file if it doesn't exist)
fileID = fopen(outputFilePath, 'w');

% Check if the file opened successfully
if fileID == -1
    error('Could not open file for writing.');
end

% Write each complex number in concatenated_signal to the file
for i = 1:length(concatenated_signal)
    realPart = real(concatenated_signal(i));  % Extract the real part of the complex number
    imagPart = imag(concatenated_signal(i));  % Extract the imaginary part of the complex number
    
    % Write the real and imaginary parts as a complex number in the format "real + imagi"
    fprintf(fileID, '%.6f + %.6fi\n', realPart, imagPart);
end

% Close the file
fclose(fileID);
% Constants
numChirps = 768;              % Number of chirps (64 loops * 12 chirps/loop)
numSamplesPerChirp = 256;      % Number of ADC samples per chirp
Nfft_range = 320;              % FFT size for the range dimension
Nfft_doppler = 13;             % FFT size for the Doppler dimension

% Radar Parameters
range_res = c / (2 * sweepBandwidth);                         % Range resolution (meters)
max_range = range_res * (Nfft_range - 1);                     % Maximum measurable range
doppler_res = 1 / (numChirpPerLoop * chirpDuration);          % Doppler resolution (Hz)
max_doppler = doppler_res * (Nfft_doppler / 2);               % Maximum Doppler shift (Hz)

% Compute the Short-Time Fourier Transform (STFT) on the concatenated signal
[~, f, t, ps] = spectrogram(concatenated_signal(1:256), windowSize, overlap, Nfft_stft, samplingRate);

% Convert power spectrum to decibels (dB)
ps_dB = 10*log10(abs(ps));

% Plot the spectrogram
figure;
imagesc(t, f/1e6, ps_dB);  % Divide frequency by 1e6 to convert to MHz
axis xy;
xlabel('Time (s)');
ylabel('Frequency (MHz)');
title('Spectrogram of a Chirp');
colorbar;

% Function to read binary file data
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
