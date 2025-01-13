% Define file path and parameters
fileFullPath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\metal_plate_distance_3m\master_0000_data.bin';
frameIdx = 49;             % Index of the frame to read
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 64;              % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade (if needed)
fc = 77e9;                  % Radar operating frequency (77 GHz)
c = 3e8;                    % Speed of light (m/s)
sweepBandwidth = 3.16e9;     % Bandwidth of the FMCW radar sweep (3.16 GHz)
chirpDuration = 40e-6;       % Chirp duration (40 microseconds)
samplingRate = 8000e3;       % Sampling rate (8000 ksps)

% Read radar data
adcDataComplex = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);

% Select antenna index and extract chirp ADC matrix
antennaIdx = 1;  
chirp_ADC_matrix = squeeze(adcDataComplex(:, :, antennaIdx, :));

% Display matrix size (should be [numSamplePerChirp, numLoops, numChirpsPerLoop])
disp(size(chirp_ADC_matrix));

% Extract first chirp of the first loop
first_chirp_first_loop = chirp_ADC_matrix(:, 1, 1);
disp(size(first_chirp_first_loop));  % Should be [numSamplePerChirp, 1]
disp(first_chirp_first_loop);

% Compute mean across all chirps (64 loops * 12 chirps per loop)
mean_chirp_signal = mean(reshape(chirp_ADC_matrix, numSamplePerChirp, []), 2);
disp(size(mean_chirp_signal));
%disp(mean_chirp_signal);

% STFT parameters
windowSize = 32;        % Window size for STFT
overlap = 8;            % Overlap between windows
Nfft_stft = 256;         % Number of FFT points for STFT

% Compute STFT
[stft_data, f, t] = stft(mean_chirp_signal, samplingRate, 'Window', hamming(windowSize), 'OverlapLength', overlap, 'FFTLength', Nfft_stft);

% Convert to magnitude in dB
stft_magnitude_dB = 20 * log10(abs(stft_data));

% Plot STFT results
figure;
imagesc(t, f/1e6, stft_magnitude_dB);  % Convert frequency to MHz
axis xy;
xlabel('Time (s)');
ylabel('Frequency (MHz)');
title('STFT-based Spectrogram (Mean of Chirps)');
colorbar;

% Function to read binary radar data
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