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
%   obj: object instance of calibrationCascade


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
        %disp("The size of all data from one folder");
        %disp(size([radar_data_Rxchain]));
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
    %%%%%%%%%%%%%%%%%%% We can take the radar matrix for our works and return it through .csv file 
%     disp("Outputdata_structure"); %The output datastructure
%     disp(size( outData));
%     % Select antenna index and extract chirp ADC matrix
%     antennaIdx = 4;  
%     chirp_ADC_matrix = outData(:, :, antennaIdx, :);
%     chirp_ADC_matrix = squeeze(chirp_ADC_matrix);
%     fprintf('The chirp signals array size is [%s]\n', num2str(size(chirp_ADC_matrix)));
% 
%     % Extract chirps signal from one TX
%     chirps_signal = chirp_ADC_matrix(:, :, 1); % This array consists of 128 chirps, each having 256 complex ADC samples
%     %disp(size(chirps_signal));% Should be [256, 128]
%     chirps_signal=transpose(chirps_signal);
%     %disp(size(chirps_signal));% Should be [128, 256]
%     %disp(length(chirps_signal(1,:)));% Should be [128, 256]
%     %disp((chirps_signal(1,:)));% Should be [128, 256]
%     % Define the file path where the complex values will be saved
%     filePath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\chirps_signal_3.csv';
% 
%     % Open the file in write mode
%     fileID = fopen(filePath, 'w');
%     if fileID == -1
%         error('Failed to create the CSV file.');
%     end
%     %disp(size(chirps_signal, 1));
%     % Write the chirps signal matrix to the file
%     for i = 1:128
%         chirp_data = chirps_signal(i, :); % Extract the i-th chirp
%         % Convert complex numbers to a string format: "real+imag*j"
%         chirp_string = arrayfun(@(x) sprintf('%.4f%+.4fj', real(x), imag(x)), chirp_data, 'UniformOutput', false);
%         % Join the strings with commas and write to the file
%         fprintf(fileID, '%s\n', strjoin(chirp_string, ','));
%     end
%     
%     % Close the file
%     fclose(fileID);
% 
%     disp(['File written to: ', filePath]);
end

%re-order the RX channels so that it correspond to the channels of
% ********   16_lamda     ****   4_lamda    ****
% and only maintain RX/TX data used for requred MIMO analysis.
% outData = outData(:,:,RxForMIMOProcess,IdTxForMIMOProcess);
% outData = outData(:,:,RxForMIMOProcess,:);



