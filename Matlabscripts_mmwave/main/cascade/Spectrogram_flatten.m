filePaths = {
%     "F:\20241210\20241210_hexacopter_ang1\slave1_0000_data.bin",
%     "F:\20241210\20241210_kaffir_ang1\slave1_0000_data.bin",
% 	"F:\20241210\20241210_phantom_ang1\slave1_0000_data.bin",
% 	"F:\20241210\20241210_Rcplane_ang1\slave1_0000_data.bin",
% 	"F:\20241210\20241210_environment1\slave1_0000_data.bin",
% 	"F:\20241210\20241210_environment2\slave1_0000_data.bin",
% 	"F:\20241210\20241210_environment3\slave1_0000_data.bin",
% 
% 	"F:\20241210\20241210_hexacopter_ang2\slave1_0000_data.bin",
% 	"F:\20241210\20241210_kaffir_ang2\slave1_0000_data.bin",
% 	"F:\20241210\20241210_phantom_ang2\slave1_0000_data.bin",
% 	"F:\20241210\20241210_Rcplane_ang2\slave1_0000_data.bin",
% 
% 	"F:\20241210\20241210_hexacopter_ang3\slave1_0000_data.bin",
% 	"F:\20241210\20241210_kaffir_ang3\slave1_0000_data.bin",
% 	"F:\20241210\20241210_phantom_ang3\slave1_0000_data.bin",
	"F:\20241210\20241210_Rcplane_ang3\slave1_0000_data.bin",

% 	"F:\20241210\20241210_hexacopter_ang4\slave1_0000_data.bin",
% 	"F:\20241210\20241210_kaffir_ang4\slave1_0000_data.bin",
% 	"F:\20241210\20241210_phantom_ang4\slave1_0000_data.bin", 
% 	"F:\20241210\20241210_Rcplane_ang4\slave1_0000_data.bin"

%     "F:\20241210\20241210_hexacopter_ang5\slave1_0000_data.bin",
%     "F:\20241210\20241210_hexacopter_ang6\slave1_0000_data.bin",
%     "F:\20241210\20241210_hexacopter_ang7\slave1_0000_data.bin",
%     "F:\20241210\20241210_hexacopter_ang8\slave1_0000_data.bin",
%     "F:\20241210\20241210_hexacopter_ang9\slave1_0000_data.bin",
%     "F:\20241210\20241210_hexacopter_ang10\slave1_0000_data.bin",
%     
%     "F:\20241210\20241210_phantom_ang5\slave1_0000_data.bin",
%     "F:\20241210\20241210_phantom_ang6\slave1_0000_data.bin",
%     "F:\20241210\20241210_phantom_ang7\slave1_0000_data.bin",
%     "F:\20241210\20241210_phantom_ang8\slave1_0000_data.bin",
%     "F:\20241210\20241210_phantom_ang9\slave1_0000_data.bin",
%     "F:\20241210\20241210_phantom_ang10\slave1_0000_data.bin",
%     
%     "D:\20241210 extension\20241210_kaffir_ang5\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_kaffir_ang6\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_kaffir_ang7\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_kaffir_ang8\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_kaffir_ang9\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_kaffir_ang10\slave1_0000_data.bin",
%     
%     "D:\20241210 extension\20241210_Rcplane_ang5\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_Rcplane_ang6\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_Rcplane_ang7\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_Rcplane_ang8\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_Rcplane_ang9\slave1_0000_data.bin",
%     "D:\20241210 extension\20241210_Rcplane_ang10\slave1_0000_data.bin"
    
};

numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 127;              % Number of loops per frame,

numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade (adjust based on your setup)
% Constants (adjust based on your radar parameters)
fc = 77e9;                  % Radar operating frequency (77  GHz for mmWave radar)
c = 3e8;                    % Speed of light (m/s)
sweepBandwidth = 3.16e9;     % Bandwidth of the FMCW radar sweep (3.16 GHz)
chirpDuration = 40e-6;       % Chirp duration (40 microseconds)
fs     = 10e6;
frameDuration = chirpDuration*numLoops; % 40e-3
T = chirpDuration; % PRI
PRF    = 1/T;
lambda = c/fc;
slope = sweepBandwidth/T;
F = numSamplePerChirp/T;
Vmax = lambda/(T*4);
Rmax = F*c/(2*slope); % TI's MIMO Radar doc

wav = phased.RectangularWaveform('SampleRate',fs,'PulseWidth',2e-6,'PRF',PRF);
% FFT parameters
% Nfft_range = 320;           % Number of FFT points for range dimension
% Nfft_doppler = 97;
Nfft_doppler = 2^(ceil(log2(numLoops)));
Nfft_range = 2^(ceil(log2(numSamplePerChirp)));

% Create the filter for zero Doppler clutter removal
cutoff_freq = 10;  % Set cutoff frequency for the high-pass filter (Hz)
sampling_freq = 10^6;  % Sampling frequency (example value, should match your data)
order = 6;  % Order of the Butterworth filter

% Design a Butterworth high-pass filter
[b, a] = butter(order, cutoff_freq / (sampling_freq / 2), 'high');
% Sampling rate
fs = 10e6; % 10 MHz

numFrames =1;



scriptDir = "E:\OneDrive - University of Moratuwa\Desktop\FYP\Spectrogram dataset";
csvFilePath = fullfile(scriptDir, 'signal_data.csv'); % Path to save the CSV file

% Open the file for writing (create a new one or overwrite if it exists)
fileID = fopen(csvFilePath, 'w');

for i = 1:length(filePaths)
    fileFullPath = filePaths{i};
    name = extractBetween(fileFullPath, "F:\20241210\20241210_", "\slave1");
    name= name + " slave1";
    % Replace underscores with spaces
    name = strrep(name, '_', ' ');
    
    saveDir = fullfile(scriptDir, name);
    
    % Create folder if it does not exist
    if ~exist(saveDir, 'dir')
        mkdir(saveDir);
    end
    
    for frameIdx = 2:128
        for antenna_idx = 3:4
            subName = name + " " + string(frameIdx)+" chn"+string(antenna_idx);
            mmap_array = readBinFile(fileFullPath, frameIdx, numSamplePerChirp, numChirpPerLoop, numLoops, numRXPerDevice);
            rBin = 1:256;
            nfft = 2^12;window = 64;noverlap = 50;shift = window - noverlap;
            %nfft = 2^12;window = 256;noverlap = 200;shift = window - noverlap;
            %signal=sum(fft(squeeze(mmap_array(rBin,:,1))));
            radar_cube1 = squeeze(mmap_array(rBin,:,antenna_idx));
            signal = radar_cube1(:).';
            
            % Write the signal data as a row in the CSV file
            fprintf(fileID, '%f,', signal); % Write data separated by commas
            fprintf(fileID, '\n');          % Move to the next row
            
            sx1 = myspecgramnew(signal,window,nfft,shift); % mti filter and IQ correction
            sx2 = abs(flipud(fftshift(sx1,1)));
            timeAxis = (1:numFrames)*frameDuration; % Time
            freqAxis = linspace(-PRF/2,PRF/2,nfft); % Frequency Axis
            fig = figure('visible','on');
            colormap(jet(256));
            % set(gca,'units','normalized','outerposition',[0,0,1,1]);
            doppSignMTI = imagesc(timeAxis,[-PRF/2 PRF/2],20*log10(abs(sx2/max(sx2(:)))));
            %     axis xy
            %     set(gca,'FontSize',10)
            disp(name);
            title(subName);
            %     title(fOut(end-22:end-4))
            xlabel('Time (sec)');
            ylabel('Frequency (Hz)');
            
            c = colorbar;
            c.Label.String = 'Power (dB)';
            caxis([-30 0]) % 40
            set(gca, 'YDir','normal')
            set(gcf,'color','w');
            
            % Save figure
            % Sanitize the 'name' variable to make it a valid filename
            sanitizedName = regexprep(subName, '[^\w]', '_'); % Replace non-word characters with underscores
            sanitizedName = strrep(sanitizedName, ' ', '_'); % Replace spaces with underscores
            filePath = fullfile(saveDir, [sanitizedName + '.png']);
            saveas(fig, filePath);
            close(fig);
        end
    end
end
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
