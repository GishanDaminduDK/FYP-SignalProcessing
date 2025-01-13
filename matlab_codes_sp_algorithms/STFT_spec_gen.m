fileFullPath = "D:\Drone-Swarm-Detection-with-AWR2243\Our data\Radar_Data\data\data\phanton_60_degree(left_side)_same_height_\master_0000_data.bin";

frameIndices = [2:4];  % List of frame indices
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 128;              % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade
fc = 77e9;                  % Radar operating frequency (77 GHz for mmWave radar)
c = 3e8;                    % Speed of light (m/s)
sweepBandwidth = 3.16e9;     % Bandwidth of the FMCW radar sweep (3.16 GHz)
chirpDuration = 40e-6;       % Chirp duration (40 microseconds)

% FFT parameters
Nfft_doppler = 2^(ceil(log2(numLoops)));
Nfft_range = 2^(ceil(log2(numSamplePerChirp)));

% Create the filter for zero Doppler clutter removal
cutoff_freq = 10;  % Set cutoff frequency for the high-pass filter (Hz)
sampling_freq = 1000;  % Sampling frequency (should match your data)
order = 6;  % Order of the Butterworth filter
[b, a] = butter(order, cutoff_freq / (sampling_freq / 2), 'high');

fs = 10e6; % Sampling rate (10 MHz)

% Define the folder path for saving the images
outputFolder = fullfile(fileparts(fileFullPath), 'train_phantom_4');

% Check if the folder exists, if not, create it
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

% Loop through each frame index
for frameIdx = frameIndices
    % Read binary file data
    adcData1Complex = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
    
    antennaIdx = 3;  % Antenna number (2nd antenna)
    
    % Extract the chirp ADC matrix for all loops and the selected antenna
    chirp_ADC_matrix = adcData1Complex(:, :, antennaIdx, :);
    
    % Remove singleton dimensions and apply high-pass filter
    RDC = squeeze(chirp_ADC_matrix(:,:,1));
    RDC = filtfilt(b, a, RDC);  % Apply high-pass filter
    
    % Reshape RDC into a 1D array for further processing
    radar_data = reshape(RDC(:, :), [numel(RDC(:, :)), 1]);
    
    % Reshape radar_data into an 8x4096 matrix
    reshaped_data = reshape(radar_data, 4096, 8);

    % Process each subarray
    for i = 1:8
        radar_data1 = reshaped_data(:, i);
    
        % Perform STFT computation
        [stft_data, f, t] = stft(radar_data1, fs, 'Window', hamming(128), 'OverlapLength', 100, 'FFTLength', 128);

        % Convert magnitude to dB
        stft_data_dB = 20 * log10(abs(stft_data));

        % Plot the STFT-based spectrogram
        figure('Position', [100, 100, 600, 450], 'PaperPositionMode', 'auto');  % Adjust figure size (larger for higher resolution)

        surf(t, f, stft_data_dB, 'EdgeColor', 'none');
        axis tight;
        view(0, 90);

        % Set colormap
        colormap('jet');

        % Set color scale
        caxis([-10 70]);

        % Remove axis labels, ticks, and white borders
        set(gca, 'XTick', [], 'YTick', [], 'XColor', 'none', 'YColor', 'none');
        set(gca, 'LooseInset', [0, 0, 0, 0]);

        % Save the final frame of the STFT spectrogram with no border and no axis labels
        % Increasing resolution by setting the DPI (dots per inch)
        exportgraphics(gcf, fullfile(outputFolder, sprintf('phantom4_frame_%d_subarray_%d.png', frameIdx, i)), 'BackgroundColor', 'none', 'Resolution', 224, 'ContentType', 'image');

        % Close the figure to avoid clutter
        close(gcf);
    end
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

% Function to preprocess the spectrogram
function S = helperPreProcess(S)
%helperPreProcess converts each spectrogram into log-scale and normalizes each log-scale spectrogram
%to [0,1].

    S = 20*log10(abs(S)); % logarithmic scaling to dB
    for ii = 1:size(S,3)
        zs = S(:,:,ii);
        zs = (zs - min(zs(:)))/(max(zs(:))-min(zs(:))); % normalize amplitudes of each map to [0,1]
        S(:,:,ii) = zs;
    end
end
