%  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/ 
%  
%  
%   Redistribution and use in source and binary forms, with or without 
%   modification, are permitted provided that the following conditions 
%   are met:
%  
%     Redistributions of source code must retain the above copyright 
%     notice, this list of conditions and the following disclaimer.
%  
%     Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the 
%     documentation and/or other materials provided with the   
%     distribution.
%  
%     Neither the name of Texas Instruments Incorporated nor the names of
%     its contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.
%  
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
%   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
%   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
%   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
%   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
%   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
%   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
%   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
%   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
%   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
%   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%  

%datapath.m
%
% datapath function of calibrationCascade module, this function calibrates the ADC data with the calibration
% matrix installed with the path name given by calibrationfilePath.
% Calibration is done directly on the raw ADC data before any further
% processing. Apply frequency and phase calibration in time domain; amplitude
% calibration is optional, can be turned on or off
%
%input
%obj: object instance of calibrationCascade


function outData = datapath(obj)

%load calibration file
load(obj.calibrationfilePath);
RangeMat = calibResult.RangeMat;
PeakValMat = calibResult.PeakValMat;

fileFolder = obj.binfilePath;
frameIdx = obj.frameIdx;

numSamplePerChirp = obj.numSamplePerChirp;
nchirp_loops = obj.nchirp_loops;
numChirpsPerFrame = obj.numChirpsPerFrame;
TxToEnable = obj.TxToEnable;
Slope_calib = obj.Slope_calib;
fs_calib = obj.fs_calib;
Sampling_Rate_sps = obj.Sampling_Rate_sps;

chirpSlope = obj.chirpSlope;
calibrationInterp = obj.calibrationInterp;
TI_Cascade_RX_ID = obj.TI_Cascade_RX_ID;
RxForMIMOProcess = obj.RxForMIMOProcess;
IdTxForMIMOProcess = obj.IdTxForMIMOProcess;
numRX = obj.numRxToEnable;
phaseCalibOnly = obj.phaseCalibOnly;
adcCalibrationOn = obj.adcCalibrationOn;
N_TXForMIMO = obj.N_TXForMIMO;
NumAnglesToSweep =  obj.NumAnglesToSweep ;
RxOrder = obj.RxOrder;
NumDevices = obj.NumDevices;

numTX = length(TxToEnable);
outData = [];


fileName=[fileFolder];
switch obj.dataPlatform

    case 'TDA2'
        numChirpPerLoop = obj.numChirpsPerFrame/obj.nchirp_loops; 
        numLoops = obj.nchirp_loops;             
        numRXPerDevice = 4; % Fixed number      
        [radar_data_Rxchain] = read_ADC_bin_TDA2_separateFiles(fileName,frameIdx,numSamplePerChirp,numChirpPerLoop,numLoops, numRXPerDevice, 1);
        disp("The size of all data from one folder");
        disp(size([radar_data_Rxchain]));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%  Write CSV files %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
%         data = radar_data_Rxchain; % Example 4D complex array
% 
%         % Reshape the 4D array into a 2D array (256*128 rows, 16*12 columns)
%         data_reshaped = reshape(data, [256*128, 16*12]);
% 
%         % Separate real and imaginary parts
%         real_part = real(data_reshaped);
%         imag_part = imag(data_reshaped);
% 
%         % Combine real and imaginary parts into a single 2D array
%         data_combined = [real_part, imag_part];
% 
%         % Write the 2D array to a CSV file
%         writematrix(data_combined, 'D:\drone_swarms_stationary\four_drones\rcplane8_20.csv');
%         disp('4D array saved as 2D to complex_data_2D.csv');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% Read CSV files %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Read the 2D combined data from the CSV file
%         data_combined = readmatrix('D:\drone_swarms_stationary\four_drones\drone_swarm2.csv');
% 
%         %Determine sizes
%         num_real_cols = size(data_combined, 2) / 2; % Split columns between real and imaginary parts
%         rows = 256 * 128;
%         cols = 16 * 12;
% 
%         %Separate real and imaginary parts
%         real_part = data_combined(:, 1:num_real_cols);
%         imag_part = data_combined(:, num_real_cols+1:end);
% 
%         %Reconstruct the 2D complex array
%         data_reshaped = complex(real_part, imag_part);
% 
%         %Reshape back to the original 4D array
%         data_reconstructed = reshape(data_reshaped, [256, 128, 16, 12]);
%         radar_data_Rxchain = data_reconstructed;
%         disp('4D array reconstructed from complex_data_2D.csv');



    otherwise
        error('Not supported data capture platform!'); 
        
end


%use the first TX as reference by default
TX_ref = TxToEnable(1);

if adcCalibrationOn == 0
    outData = radar_data_Rxchain;
else
    
    for iTX = 1: numTX
        
        %use first enabled TX1/RX1 as reference for calibration
        TXind = TxToEnable(iTX);
%         TXind = iTX;
        %construct the frequency compensation matrix             
        freq_calib = (RangeMat(TXind,:)-RangeMat(TX_ref,1))*fs_calib/Sampling_Rate_sps *chirpSlope/Slope_calib;       
        freq_calib = 2*pi*(freq_calib)/(numSamplePerChirp * calibrationInterp);
        correction_vec = (exp(1i*((0:numSamplePerChirp-1)'*freq_calib))');
        freq_correction_mat = repmat(correction_vec, 1, 1, nchirp_loops);
        freq_correction_mat = permute(freq_correction_mat, [2 3 1]);
        %disp("Frequency correction matrix");
        %disp(size(freq_correction_mat));
        outData1TX = radar_data_Rxchain(:,:,:,iTX).*freq_correction_mat;
        %construct the phase compensation matrix
        phase_calib = PeakValMat(TX_ref,1)./PeakValMat(TXind,:);
        %remove amplitude calibration
        if phaseCalibOnly == 1
            phase_calib = phase_calib./abs(phase_calib);
        end
        phase_correction_mat = repmat(phase_calib.', 1,numSamplePerChirp, nchirp_loops);
        phase_correction_mat = permute(phase_correction_mat, [2 3 1]);
        outData(:,:,:,iTX) = outData1TX.*phase_correction_mat;
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%% Add phase shift to data (Change azimuth )%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Radar parameters
    % Load the MAT file containing the radar data cube
    radar_data=outData;
    numADCSamples=256;
    numChirps=128;
    numReceivers=16;
    numTransmitters=12;
    % Reshape the data cube to 3D: 256 (ADC samples) x 128 (chirps) x 192 (channels)
    data3D = reshape(radar_data, numADCSamples, numChirps, numReceivers * numTransmitters);

    % Define parameters for range shift
    c = 3e8; % Speed of light (m/s)
    B = 1e9; % Bandwidth of the radar (Hz) - modify based on your radar system
    Rmax = 15.19; % Maximum range (m) - modify as needed
    delta_r = 1; % Desired range shift (m) - modify this value as needed

    % Compute the phase shift for range shift
    n = 0:numADCSamples-1; % ADC sample indices
    phase_shift = exp(-1j * 2 * pi * n * delta_r / Rmax);

    % Apply the phase shift to the 3D matrix
    shifted_data3D = zeros(size(data3D)); % Initialize shifted data matrix
    for chirpIdx = 1:numChirps
        for channelIdx = 1:(numReceivers * numTransmitters)
            shifted_data3D(:, chirpIdx, channelIdx) = data3D(:, chirpIdx, channelIdx) .* phase_shift.'; % Apply phase shift
        end
    end

   

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    Theta = pi / 2; % Example phase shift (90 degrees)
    numAntennas = 192; % Total number of antennas (M)

    % Reshape the data to (256, 128, 192) for processing
    outData = reshape(outData, 256, 128, []);
    %%%%%%%%%%%To give azimuth angle change code part
    % Calculate phase shifts and apply to radar data
    for i = 1:numAntennas
        % Generate the antenna indices
        m = 0:(size(outData, 3) - 1); % Adjusted to match the 3rd dimension

        % Calculate phase shifts for each antenna
        phase_shifts = 0.5 * pi * m; % Phase shifts in radians

        % Apply phase shifts to the radar data
        outData(:, :, i) = outData(:, :, i) .* exp(1j * phase_shifts(i)*0);
    end

    % Reshape back to the original size (256, 128, 16, 12)
    %outData = reshape(outData, 256, 128, 16, 12);
    
    % Radar parameters
   
   % Define parameters
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%Velocity Shift %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    velocity_shift = 2000000; % Velocity shift in m/s
    numAntennas = 192; % Total number of antennas
    slope = 78e6; % Slope in Hz/second (78 MHz/microsecond)
    chirp_duration = 40e-6; % Duration of a single chirp in seconds
    n_chirps = 128; % Number of chirps
    chirp_samples = 256; % Number of samples per chirp

    % Reshape outData to (256, 128, 192) if not already
    outData = reshape(outData, chirp_samples, n_chirps, numAntennas);

    % Create the time vector for phase shift calculation (column vector)
    time = (0:n_chirps-1)' * chirp_duration; 

    % Calculate the frequency shift due to velocity
    frequency_shift = 2 * slope * velocity_shift / (3e8); % Speed of light in m/s

    % Calculate the phase shift for each chirp (column vector)
    phase_shift = exp(1j * 2 * pi * frequency_shift * time); % Size: (128, 1)

    % Expand phase_shift to match dimensions of outData(:, :, j) for broadcasting
    phase_shift = repmat(phase_shift.', chirp_samples, 1); % Size: (256, 128)

    % Apply phase shift to each antenna
    for j = 1:numAntennas
        outData(:, :, j) = outData(:, :, j) .* phase_shift; % Explicit broadcasting
    end

    % Reshape back to original dimensions (256, 128, 16, 12)
    outData = reshape(outData, chirp_samples, n_chirps, 16, 12);

    % Display the final size of outData
    disp('OutData complete. Size is:');
    disp(size(outData));


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%% Generate Spectrogram %%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    radarDataChain=outData;
    samplesPerChirp = 256;    % Number of samples per chirp
    chirpsPerLoop = 12;       % Number of chirps per loop
    loopsPerFrame = 128;      % Number of loops per frame

    rxChannelsPerDevice = 4;  % Number of receiving channels per device
    numDevicesCascade = 4;    % Number of devices in the cascade (adjust based on your setup)
    % Constants (adjust based on your radar parameters)
    radarFrequency = 77e9;    % Radar operating frequency (77 GHz for mmWave radar)
    speedOfLight = 3e8;       % Speed of light (m/s)
    radarBandwidth = 3.16e9;  % Bandwidth of the FMCW radar sweep (3.16 GHz)
    chirpTime = 40e-6;        % Chirp duration (40 microseconds)
    sampleRate = 10e6;
    frameTime = chirpTime * loopsPerFrame; % 40e-3
    pulseRepetitionInterval = chirpTime; % PRI
    pulseRepetitionFrequency = 1 / pulseRepetitionInterval;
    wavelength = speedOfLight / radarFrequency;
    slopeFactor = radarBandwidth / pulseRepetitionInterval;
    sampleFreq = samplesPerChirp / pulseRepetitionInterval;
    velocityMax = wavelength / (pulseRepetitionInterval * 4);
    rangeMax = sampleFreq * speedOfLight / (2 * slopeFactor); % TI's MIMO Radar doc

    waveform = phased.RectangularWaveform('SampleRate', sampleRate, 'PulseWidth', 2e-6, 'PRF', pulseRepetitionFrequency);
    % FFT parameters
    fftPointsDoppler = 2^(ceil(log2(loopsPerFrame)));
    fftPointsRange = 2^(ceil(log2(samplesPerChirp)));
    mappedArray = radarDataChain;
    rangeBins = 1:256;
    nfft = 2^12;
    windowSize = 64;
    overlapSize = 50;
    shiftSize = windowSize - overlapSize;
    radarCube = squeeze(mappedArray(rangeBins, :, 4, 1));
    signalData = radarCube(:).';
    totalFrames = 1;
    specgramOutput = myspecgramnew(signalData, windowSize, nfft, shiftSize); % mti filter and IQ correction
    specgramAbs = abs(flipud(fftshift(specgramOutput, 1)));
    timeVector = (0:totalFrames) * frameTime; % Time
    freqVector = linspace(-pulseRepetitionFrequency / 2, pulseRepetitionFrequency / 2, nfft); % Frequency Axis
    % Corrected MATLAB code
    figureHandle = figure(3); % Specify figure number
    set(figureHandle, 'Visible', 'on'); % Make the figure visible
    colormap(jet(256)); % Set colormap

    % Plot the spectrogram
    imagesc(timeVector, [-pulseRepetitionFrequency / 2, pulseRepetitionFrequency / 2], ...
            20 * log10(abs(specgramAbs / max(specgramAbs(:)))));
    title("Spectrograms of four same type rcplane drone swarms");
    xlabel('Time (sec)');
    ylabel('Frequency (Hz)');

    % Add colorbar and set its label
    colorBarHandle = colorbar;
    colorBarHandle.Label.String = 'Power (dB)';

    % Set color axis limits and adjust plot orientation
    caxis([-30 0]); % Set color axis limits
    set(gca, 'YDir', 'normal'); % Normal y-axis direction
    set(gcf, 'Color', 'w'); % Set figure background color to white


end

%re-order the RX channels so that it correspond to the channels of
% ********   16_lamda     ****   4_lamda    ****
% and only maintain RX/TX data used for requred MIMO analysis.
% outData = outData(:,:,RxForMIMOProcess,IdTxForMIMOProcess);
% outData = outData(:,:,RxForMIMOProcess,:);



