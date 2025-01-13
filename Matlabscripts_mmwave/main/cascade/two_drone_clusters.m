filePaths = { 
    %"E:\20241210 continued\20241210_Rcplane_ang10\master_0000_data.bin",
    "D:\RadarDataset\20241210_phantom_ang8\master_0000_data.bin",
};

filePaths2 = {
    %"E:\20241210 continued\20241210_Rcplane_ang8\master_0000_data.bin", 
    "D:\RadarDataset\20241210_Rcplane_ang6\master_0000_data.bin",
    };

% Radar parameters
numSamplePerChirp = 256;  
numChirpPerLoop = 12;     
numLoops = 128;             
numRXPerDevice = 4;         
numDevices = 4;             
fc = 77e9;                  
c = 3e8;                    
sweepBandwidth = 3.16e9;   
chirpDuration = 40e-6;      
fs = 10e6;                 
frameDuration = chirpDuration * numLoops; 
T = chirpDuration;         
PRF = 1 / T;
lambda = c / fc;
slope = sweepBandwidth / T;
F = numSamplePerChirp / T;
Vmax = lambda / (T * 4);
Rmax = F * c / (2 * slope);

% High-pass filter design
Fs = 8e6; % Sampling frequency (8 MHz)
cutoff_freq = 0.3e6; % 0.3 MHz
order = 6; % Filter order
[b, a] = butter(order, cutoff_freq / (Fs / 2), 'high');

scriptDir = "D:\DroneSwarmsImages\range testing";

for i = 1:length(filePaths)
    for j = 1:length(filePaths2)
        fileFullPath = {filePaths{i}, filePaths2{j}};
        name = extractBetween(fileFullPath{1}, "D:\RadarDataset\20241210_", "\master");
        name = name + " master";
        name = strrep(name, '_', ' ');
        name2 = extractBetween(fileFullPath{2}, "D:\RadarDataset\20241210_", "\master");
        name2 = name2 + " master";
        name2 = strrep(name2, '_', ' ');

        saveDir = fullfile(scriptDir, name + " " + name2);
        if ~exist(saveDir, 'dir')
            mkdir(saveDir);
        end

        l = length(fileFullPath)
        shiftedDataAll = cell(1, l); % Store shifted data for all ranges
        for frameIdx = 2:126
            for m = 1:l 
                for antenna_idx = 1:1
                    adcData{m} = readBinFile(fileFullPath{m}, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);

                    % Reshape to 3D for processing
                    adcData{m} = reshape(adcData{m}, size(adcData{m}, 1), size(adcData{m}, 2), size(adcData{m}, 3) * size(adcData{m}, 4));

                    % Apply the high-pass filter on real and imaginary parts
                    for antennaIdx = 1:size(adcData{m}, 3)
                        realPart = real(adcData{m}(:, :, antennaIdx));
                        imagPart = imag(adcData{m}(:, :, antennaIdx));

                        filteredReal = filter(b, a, realPart, [], 1); % High-pass filter on real part
                        filteredImag = filter(b, a, imagPart, [], 1); % High-pass filter on imaginary part

                        adcData{m}(:, :, antennaIdx) = complex(filteredReal, filteredImag); % Combine filtered components
                    end

                    % Apply range shifting
                    %ranges = {0, 1.4 + (1.4) * rand()};
                    %ranges = {0, -0.5};
                    ranges = {-0.5 + (-0.2 - (-0.5)) * rand(), 1.2 + (2.0 - 1.2) * rand()};
                    range_shift = ranges{m};
                    shiftedDataRange = applyRangeShift(adcData{m}, range_shift);
                    shiftedDataAll{m} = shiftedDataRange; % Store range-shifted data

                    end
                end
                % Create combined plot
                combinedData = shiftedDataAll{1} + shiftedDataAll{2}; % Sum matrices for drone 1 and drone 2
                % selectedData = combinedData(:, startChirp:endChirp, 2);

                % Process and plot 
                totalChirps = size(shiftedDataRange, 2);
                chirpsPerGroup = 16;
                numGroups = ceil(totalChirps / chirpsPerGroup);
                for groupIdx = 1:numGroups
                        subName = name + " " + name2 + " " + string(frameIdx) + " chn" + string(antenna_idx) + " grp " + string(groupIdx) ;
                        sanitizedName = regexprep(subName, '[^\w]', '_');
                        pathName = strrep(sanitizedName, ' ', '_');
                        startChirp = (groupIdx - 1) * chirpsPerGroup + 1;
                        endChirp = min(groupIdx * chirpsPerGroup, totalChirps);

                        selectedData = combinedData(:, startChirp:endChirp, 2);

                        flattenedData = complex(double(real(selectedData(:))), double(imag(selectedData(:))));
                        realPart = real(flattenedData);

                        t = (0:length(flattenedData) - 1) / Fs;
                        [cfs_real, f_real] = cwt(realPart, Fs);

                        % Plot and save the wavelet transform
                        figure;
                        h_real = pcolor(t, f_real, abs(cfs_real).^2);
                        set(h_real, 'EdgeColor', 'none');
                        colormap jet;
                        colorbar;
                        caxis([0 35000]); % Set the color scale limits
                        xlabel('Time (s)');
                        ylabel('Frequency (Hz)');
                        title(sprintf(subName, 'Real'));
                        ylim([0 3.4e6]);
                        xlim([0 1e-4]);

                        hold on;
                        contour(t, f_real, abs(cfs_real).^2, 'LineWidth', 1, 'LineColor', 'k'); % Add contour lines
                        hold off;

                        filePath = fullfile(saveDir, strcat(pathName, ' Real', '.png'));
                        saveas(gcf, filePath);
                        close(gcf);
                end

                disp('Combined wavelet plot for multiple drones are generated and saved.');
            end
        end
end

% Helper Functions
function [adcData1Complex] = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice)
    Expected_Num_SamplesPerFrame = numSamplePerChirp * numChirpPerLoop * numLoops * numRXPerDevice * 2;
    fp = fopen(fileFullPath, 'r');
    if fp == -1
        error('File could not be opened.');
    end
    fseek(fp, (frameIdx - 1) * Expected_Num_SamplesPerFrame * 2, 'bof');
    adcData1 = fread(fp, Expected_Num_SamplesPerFrame, 'uint16');
    fclose(fp);

    neg = logical(bitget(adcData1, 16));
    adcData1(neg) = adcData1(neg) - 2^16;
    adcData1 = adcData1(1:2:end) + 1j * adcData1(2:2:end);

    adcData1Complex = reshape(adcData1, numRXPerDevice, numSamplePerChirp, numChirpPerLoop, numLoops);
    adcData1Complex = permute(adcData1Complex, [2 4 1 3]);
end

function shiftedData = applyRangeShift(adcData, range_shift)
    [N, C, A] = size(adcData); 
    slope = 79e12; 
    c = 3e8;
    n_samples = size(adcData, 1);
    sample_rate = 8e6; 
    time = (0:n_samples - 1) / sample_rate; 
    frequency_shift = 2 * slope * range_shift / c;
    phase_shift = 2 * pi * frequency_shift * time; 

    shiftedData = adcData;
    for chirpIdx = 1:C
        for rangeIdx = 1:N
            for antennaIdx = 1:A
                shiftedData(rangeIdx, chirpIdx, antennaIdx) = ...
                    adcData(rangeIdx, chirpIdx, antennaIdx) * ...
                    exp(1i * phase_shift(rangeIdx));
            end
        end
    end
end

