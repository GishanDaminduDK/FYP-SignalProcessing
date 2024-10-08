
fileFullPath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\small_drone\drone_with_propelers_inclined\master_0000_data.bin';%fileFullPath_2 ='D:\FYP_AI\small_drone\Normal_envionment\master_0000_data.bin';
frameIdx = 49;             % Index of the frame you want to read
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
[adcData1Complex] = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
[adcData1Complex] = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);

% Parameters for STFT
windowSize = 64;  % Adjust as needed
overlapLength = 32;  % 50% overlap
nfft = 256;  % Number of FFT points

% Calculate mean chirp
meanChirp = mean(adcData1Complex, [2, 3, 4]);  % Average across loops, chirps, and RX channels

% Apply STFT
[S, F, T] = spectrogram(meanChirp, hamming(windowSize), overlapLength, nfft, 1/chirpDuration, 'yaxis');

% Convert frequency to range
rangeAxis = F * c / (2 * sweepBandwidth);

% Plot the spectrogram
figure;
surf(T*1e6, rangeAxis, 10*log10(abs(S)), 'EdgeColor', 'none');
axis tight;
view(0, 90);
xlabel('Time (µs)');
ylabel('Range (m)');
title('Mean Chirp Spectrogram');
colorbar;
colormap('jet');
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

