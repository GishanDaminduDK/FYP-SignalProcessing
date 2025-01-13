% Define file path and parameters
fileFullPath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\Radar_Data\small_drone\Drone_No_properlers\master_0000_data.bin';
frameIdx = 128;              % Index of the frame to read
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 127;              % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade (if needed)
sample_rate = 10e6;          % Sampling rate (in Hz)

% Read radar data
adcDataComplex = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
%disp(adcDataComplex);

% Select antenna index and extract chirp ADC matrix
antennaIdx = 1;  
chirp_ADC_matrix = adcDataComplex(:, :, antennaIdx, :);
chirp_ADC_matrix = squeeze(chirp_ADC_matrix);
% Display matrix size (should be [numSamplePerChirp, numLoops, numChirpsPerLoop])
disp(size(chirp_ADC_matrix));

% Extract first chirp of the first loop
first_chirp_first_loop = chirp_ADC_matrix(:, 1, 1); % This array consists of 256 complex ADC samples
disp(size(first_chirp_first_loop));  % Should be [numSamplePerChirp, 1]
disp(first_chirp_first_loop);
% Calculate the mean of all chirps across both loops and chirps per loop
mean_of_all_chirps = mean(chirp_ADC_matrix, [2, 3]);
disp(size(mean_of_all_chirps));  % Should return [numSamplePerChirp, 1]
disp(mean_of_all_chirps); 
% Define the file path where the complex values will be saved
filePath = 'mean_chirp_no_drone_plasctic.txt';

% Open the file in write mode
fileID = fopen(filePath, 'w');

% Write each complex value to the file line by line
for i = 1:length(first_chirp_first_loop)
    fprintf(fileID, '%f + %fi\n', real(mean_of_all_chirps(i)), imag(first_chirp_first_loop(i)));
end

% Close the file
fclose(fileID);

disp(['File written to: ', filePath]);

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
