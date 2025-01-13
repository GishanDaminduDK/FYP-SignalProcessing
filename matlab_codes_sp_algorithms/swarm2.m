% Define file paths and frame indices
fileFullPath_1 = 'E:\Data_Set\Phanton_4_forward_01\new_two\master_0000_data.bin';
fileFullPath_2 = 'E:\Data_Set\Phanton_4_forward_01\new_two\master_0000_data.bin';
fileFullPath_3 = 'E:\Data_Set\Phanton_4_backword_01\new_three\master_0000_data.bin';
fileFullPath_4 = 'E:\Data_Set\Phanton_4_backword_01\new_three\master_0000_data.bin';
file_paths = {fileFullPath_1, fileFullPath_2, fileFullPath_3, fileFullPath_4}; % Use cell array for paths
frame_indices = [10,50,40,60]; % Use square brackets for array

% Initialize constants
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 128;             % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade
fc = 77e9;                  % Radar operating frequency (77 GHz for mmWave radar)
c = 3e8;                    % Speed of light (m/s)
sweepBandwidth = 0.899451e9; % Bandwidth of the FMCW radar sweep (3.16 GHz)
chirpDuration = 30e-6;      % Chirp duration (40 microseconds)

adc_raw_data = 0; % Initialize adc_raw_data as zero matrix with the same dimensions as adcData

% Read binary file data
for i = 1:length(file_paths)
    [adcData] = readBinFile(file_paths{i}, frame_indices(i), numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
    
    % Initialize adc_raw_data on the first iteration
    if i == 1
        adc_raw_data = zeros(size(adcData)); % Initialize to match the size of adcData
    end
    
    adc_raw_data = adc_raw_data + adcData; % Add matrices element-wise
end

% Extract the chirp ADC matrix for all loops and the selected antenna
antennaIdx = 4;
chirp_ADC_matrix = adc_raw_data(:, :, antennaIdx, :);

% Open file for writing
fid = fopen('output_data.txt', 'w');
if fid == -1
    error('Failed to open file for writing.');
end

% Write data to file
for loopIdx = 1:numLoops
    for chirpIdx = 1:numChirpPerLoop
        samples = chirp_ADC_matrix(:, loopIdx, chirpIdx);

        % Write each complex sample on a new line
        for sampleIdx = 1:numSamplePerChirp
            realPart = real(samples(sampleIdx));
            imagPart = imag(samples(sampleIdx));
            fprintf(fid, '%f + j*%f\n', realPart, imagPart);
        end
    end
end
fclose(fid);

% Process data
chirp_ADC_matrix = squeeze(chirp_ADC_matrix);
first_chirp_first_loop = chirp_ADC_matrix(:, 1, 1);

% Perform 2D FFT
Nfft_doppler = 2^(ceil(log2(numLoops)));
Nfft_range = 2^(ceil(log2(numSamplePerChirp)));
range_fft = fft(chirp_ADC_matrix, Nfft_range, 1); % Range FFT
doppler_fft = fftshift(fft(range_fft, Nfft_doppler, 2), 2); % Doppler FFT with shift
doppler_fft = squeeze(doppler_fft(:, :, 1)); % Extract first chirp for visualization

% Calculate axes
range_res = c / (2 * sweepBandwidth);
max_range = range_res * (Nfft_range - 1);
range_axis = linspace(0, max_range, Nfft_range);

lambda = c / fc;
maximum_velocity = lambda / (chirpDuration * 4);
velocity_axis = linspace(-maximum_velocity, maximum_velocity, Nfft_doppler);

velocity_Resolution  = lambda / (2 * numLoops * chirpDuration * 12);
% CFAR processing
doppler_fft = doppler_fft * velocity_Resolution;
magnitude_array = 20 * log10(abs(doppler_fft));
cfar_output = zeros(size(magnitude_array));
numTrainingCells = 12;
numGuardCells = 4;
P_fa = 1e-18;
threshold = 65;

for rangeIdx = (numTrainingCells + numGuardCells + 1):(size(magnitude_array, 1) - numTrainingCells - numGuardCells)
    for dopplerIdx = (numTrainingCells + numGuardCells + 1):(size(magnitude_array, 2) - numTrainingCells - numGuardCells)
        training_region = magnitude_array(rangeIdx - numTrainingCells - numGuardCells : rangeIdx + numTrainingCells + numGuardCells, ...
                                          dopplerIdx - numTrainingCells - numGuardCells : dopplerIdx + numTrainingCells + numGuardCells);
        training_region(numTrainingCells+1:end-numTrainingCells, numGuardCells+1:end-numGuardCells) = 0;
        noise_level = mean(training_region(training_region ~= 0));
        if magnitude_array(rangeIdx, dopplerIdx) > threshold
            cfar_output(rangeIdx, dopplerIdx) = magnitude_array(rangeIdx, dopplerIdx);
        end
    end
end

% Plot results
figure;
subplot(1, 2, 1);
imagesc(velocity_axis, range_axis, magnitude_array);
xlabel('Velocity (m/s)');
ylabel('Range (m)');
title('Original Range-Velocity Map');
colorbar;
axis xy;

subplot(1, 2, 2);
imagesc(velocity_axis, range_axis, cfar_output);
xlabel('Velocity (m/s)');
ylabel('Range (m)');
title('CFAR Detected Targets');
colorbar;
axis xy;

% Function to read binary radar data file
function [adcData1Complex] = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice)
    Expected_Num_SamplesPerFrame = numSamplePerChirp * numChirpPerLoop * numLoops * numRXPerDevice * 2;
    fp = fopen(fileFullPath, 'r');
    
    if fp == -1
        error('File could not be opened.');
    end
    
    % Move to the desired frame
    fseek(fp, (frameIdx - 1) * Expected_Num_SamplesPerFrame * 2, 'bof');
    adcData1 = fread(fp, Expected_Num_SamplesPerFrame, 'uint16');
    fclose(fp);
    
    neg = logical(bitget(adcData1, 16));
    adcData1(neg) = adcData1(neg) - 2^16;
    
    adcData1 = adcData1(1:2:end) + 1j * adcData1(2:2:end);
    
    adcData1Complex = reshape(adcData1, numRXPerDevice, numSamplePerChirp, numChirpPerLoop, numLoops);
    adcData1Complex = permute(adcData1Complex, [2 4 1 3]);
end
