% % Define file path and parameters
% fileFullPath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\metal_plate_distance_3m\master_0000_data.bin';
% frameIdx = 49;              % Index of the frame to read
% numSamplePerChirp = 256;    % Number of samples per chirp
% numChirpPerLoop = 12;       % Number of chirps per loop
% numLoops = 64;              % Number of loops per frame
% numRXPerDevice = 4;         % Number of receiving channels per device
% numDevices = 4;             % Number of devices in the cascade (if needed)
% % Read radar data
% adcDataComplex = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
% disp(adcDataComplex)
% % Select antenna index and extract chirp ADC matrix
% antennaIdx = 1;  
% chirp_ADC_matrix = (adcDataComplex(:, :, antennaIdx, :));
% disp(adcDataComplex(2,1,1,1))
% % Display matrix size (should be [numSamplePerChirp, numLoops, numChirpsPerLoop])
% disp(size(chirp_ADC_matrix));
% %disp(chirp_ADC_matrix)
% % Extract first chirp of the first loop
% first_chirp_first_loop = chirp_ADC_matrix(:, 1, 1);
% disp(size(first_chirp_first_loop));  % Should be [numSamplePerChirp, 1]
% disp(first_chirp_first_loop);
% 
% 
% % Function to read binary radar data
% function [adcData1Complex] = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice)
%     Expected_Num_SamplesPerFrame = numSamplePerChirp * numChirpPerLoop * numLoops * numRXPerDevice * 2;
%     fp = fopen(fileFullPath, 'r');
%     
%     if fp == -1
%         error('File could not be opened.');
%     end
%     
%     % Move to the desired frame in the file
%     fseek(fp, (frameIdx - 1) * Expected_Num_SamplesPerFrame * 2, 'bof');
%     adcData1 = fread(fp, Expected_Num_SamplesPerFrame, 'uint16');
%     fclose(fp);
%     
%     % Convert the 16-bit data to signed integers
%     neg = logical(bitget(adcData1, 16));
%     adcData1(neg) = adcData1(neg) - 2^16;
%     
%     % Combine the I and Q channels into complex values
%     adcData1 = adcData1(1:2:end) + 1j * adcData1(2:2:end);
%     
%     % Reshape and permute the data
%     adcData1Complex = reshape(adcData1, numRXPerDevice, numSamplePerChirp, numChirpPerLoop, numLoops);
%     adcData1Complex = permute(adcData1Complex, [2 4 1 3]);
% end
% Define file path and parameters
fileFullPath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\metal_plate_distance_3m\master_0000_data.bin';
outputFilePath = 'chirp_samples_fly_drone.txt'; % Output text file
frameIdx = 49;              % Index of the frame to read
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 64;              % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade (if needed)

% Read radar data
adcDataComplex = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);

% Select antenna index and extract chirp ADC matrix
antennaIdx = 1;  
chirp_ADC_matrix = adcDataComplex(:, :, antennaIdx, :);
first_chirp_first_loop = chirp_ADC_matrix(:, 1, 1);
disp(size(first_chirp_first_loop));  % Should be [numSamplePerChirp, 1]
disp(first_chirp_first_loop);

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
