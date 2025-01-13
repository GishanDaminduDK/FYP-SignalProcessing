fileFullPath = 'E:\Data_Set\new_Data_for_dr_madanayaka\drone_rotated_02\master_0000_data.bin';
%fileFullPath_2 ='D:\FYP_AI\small_drone\Normal_envionment\master_0000_data.bin';

frameIndices = [80];  % List of frame 
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 128;              % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade (adjust based on your setup)
% Constants (adjust based on your radar parameters)
fc = 77e9;                  % Radar operating frequency (77 GHz for mmWave radar)
c = 3e8;                    % Speed of light (m/s)
sweepBandwidth = 3.16e9;     % Bandwidth of the FMCW radar sweep (3.16 GHz)
chirpDuration = 40e-6;       % Chirp duration (40 microseconds)

% FFT parameters
Nfft_range = 320;           % Number of FFT points for range dimension
Nfft_doppler = 97;

% Sampling rate
fs = 10e6; % 10 MHz

% Loop through each frame index
for frameIdx = frameIndices
    % Read binary file data
    [adcData1Complex] = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
    
    antennaIdx = 2;  % Antenna number (2nd antenna)
    
    % Extract the chirp ADC matrix for all loops and the selected antenna
    chirp_ADC_matrix = adcData1Complex(:, :, antennaIdx, :);
    
    % Remove singleton dimensions
    RDC = squeeze(chirp_ADC_matrix);
    
    % Reshape RDC into a 1D array for the first channel (1st receiver, 1st transmitter)
    radar_data = reshape(RDC(:, :, 1), [numel(RDC(:, :, 1)), 1]);
    
    % Ensure the radar data length matches the STFT input size
    radar_data = radar_data(1:32768); % Adjust this range based on available data length
    
    % STFT computation for the power spectrogram
    [stft_data, f, t] = stft(radar_data, fs, 'Window', hamming(128), 'OverlapLength', 100, 'FFTLength', 128);
    
    % Convert magnitude to dB
    stft_data_dB = 20 * log10(abs(stft_data));
    
    % Plot the STFT-based spectrogram with dB scale
    figure;
    surf(t, f, stft_data_dB, 'EdgeColor', 'none');
    axis tight;
    view(0, 90);
    
    % Set the color map (e.g., 'jet', 'hot', 'parula')
    colormap();  % Change 'jet' to your desired colormap
    
    % Add labels and title
    xlabel('Time (s)');
    ylabel('Frequency (Hz)');
    title(['STFT: Frequency vs. Time Spectrogram (Frame ' num2str(frameIdx) ')']);
    
    % Add colorbar and label
    colorbar;
    ylabel(colorbar, 'Magnitude (dB)');
    caxis([-10 70]);
    
    % Save the final frame of the STFT spectrogram
    %saveas(gcf, sprintf('drone_Type_3_move4_2_Frame_%d.png', frameIdx));
    
    % Close the figure
    %close(gcf);
end

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
