fileFullPath_1 = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\Radar_Data\small_drone\drone_with_propelers_inclined\master_0000_data.bin';
outputFilePath = 'chirp_samples_fly_drone_6.txt'; % Output text file
%fileFullPath_2 ='D:\FYP_AI\small_drone\Normal_envionment\master_0000_data.bin';
frameIdx = ;             % Index of the frame you want to read
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
antennaIdx = 1; 
% FFT parameters
Nfft_range = 320;           % Number of FFT points for range dimension
Nfft_doppler = 97;  
% Read binary file data
[adcData1Complex] = readBinFile(fileFullPath_1 , frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
%[adcData1Complex_2] = readBinFile(fileFullPath_2, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
% Subtract the data from the two environments
antennaIdx=4;
% Extract the chirp ADC matrix for all loops and the selected antenna
chirp_ADC_matrix = adcData1Complex(:, :, antennaIdx, :);
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

disp(['Chirp samples have been written to ', outputFilePath]);

% Remove singleton dimensions
chirp_ADC_matrix = squeeze(chirp_ADC_matrix);
%disp(size(chirp_ADC_matrix))
first_chirp_first_loop = chirp_ADC_matrix(:, 1, 1);
%disp(size(first_chirp_first_loop));  % Should be [numSamplePerChirp, 1]
%disp(first_chirp_first_loop);


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
magnitude_array = 20*log10(abs(doppler_fft));
% Create a logical array that checks if values in columns 48, 49, or 50 exceed 110
rows_exceeding_110 = find(magnitude_array(:, 48) > 115 | magnitude_array(:, 49) > 115 | magnitude_array(:, 50) > 115);
% Find the rows where values in columns 48, 49, or 50 are less than 110
% Find the rows where values in columns 48, 49, or 50 exceed 110

% Display the row numbers and corresponding values in columns 48, 49, and 50
for i = 1:length(rows_exceeding_110)
    row = rows_exceeding_110(i);
    fprintf('Row: %d, Col 48: %.2f, Col 49: %.2f, Col 50: %.2f\n', row, ...
        magnitude_array(row, 48), magnitude_array(row, 49), magnitude_array(row, 50));
end

rows_to_replace = magnitude_array(:, 48) < 110 | magnitude_array(:, 49) < 110 | magnitude_array(:, 50) < 110;

% Replace the values in those rows in columns 48, 49, and 50 with 80
magnitude_array(rows_to_replace, 48:50) = 70;

% Display the row numbers
disp(rows_exceeding_110);

% disp(size(magnitude_array))
% disp(size(magnitude_array(:, 1:49)))
% disp(magnitude_array(:, 49))
% magnitude_array(:, 49)=80;
% magnitude_array(:, 48)=80;
% magnitude_array(:, 50)=80;
% magnitude_array(120:122, 49) =120;
% 
% disp(size(magnitude_array(:, 49)))


% Plot the Range-Velocity map (absolute value of FFT)
figure;
imagesc(velocity_axis, range_axis,20*log10(abs(doppler_fft))); % Convert to dB for better visualization
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
