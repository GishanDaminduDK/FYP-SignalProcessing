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
%

% cascade_MIMO_signalProcessing.m
%
% Top level main test chain to process the raw ADC data. The processing
% chain including adc data calibration module, range FFT module, DopplerFFT
% module, CFAR module, DOA module. Each module is first initialized before
% actually used in the chain.

clearvars
close all
homeDir = getenv('CASCADE_SIGNAL_PROCESSING_CHAIN_MIMO');
addpath(genpath([homeDir,'/modules']));
addpath(genpath([homeDir,'/main']));
addpath([homeDir,'/utils/math']);
addpath([homeDir,'/utils/dataParse']);
addpath([homeDir,'/utils/disp']);
addpath([homeDir,'/utils/cascade_json_parser']);
PLOT_ON = 1; % 1: turn plot on; 0: turn plot off
LOG_ON = 1; % 1: log10 scale; 0: linear scale
numFrames_toRun = 10; %number of frame to run, can be less than the frame saved in the raw data
SAVEOUTPUT_ON = 0;
PARAM_FILE_GEN_ON = 1;
DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP = 0 ; % Will make things slower
dataPlatform = 'TDA2';
% folder_paths_of_data = [
%     "D:\RadarDataset\20241210_hexacopter_ang4\", 
%     "D:\RadarDataset\20241210_hexacopter_ang5\", 
%     "D:\RadarDataset\20241210_hexacopter_ang6\",
%     "D:\RadarDataset\20241210_hexacopter_ang7\",
%     "D:\RadarDataset\20241210_hexacopter_ang8\"
% ];
folder_paths_of_data = [
    "D:\RadarDataset\Flying_Drone_Data\3GHz_parameters\phanton3\"
];
% Initialize five separate arrays
radar_data_1 = [];
radar_data_2 = [];
radar_data_3 = [];
radar_data_4 = [];
radar_data_5 = [];
for i=1:length(folder_paths_of_data)
    
    %% get the input path and testList
    pro_path = getenv('CASCADE_SIGNAL_PROCESSING_CHAIN_MIMO');
    input_path = strcat(pro_path,'\main\cascade\input\');
    %testList = strcat(input_path,'testList.txt');
    % Set necessary input information directly in the code
    
    name=folder_paths_of_data(i);
    disp(name);
    dataFolder_test=char(name);
    %dataFolder_test = 'D:\RadarDataset\20241210_hexacopter_ang4\'
    %dataFolder_test = sprintf('D:\\RadarDataset\\20241210_hexacopter_ang%d\\', i);
    disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    disp(dataFolder_test);
    dataFolder_calib = 'C:\Users\user\Desktop\Matlabscripts\input\calibrateResults_high.mat';
    module_param_file = 'C:\Users\user\Desktop\Matlabscripts\paramGen\module_param.m';
    %path for input folder
    %fidList = fopen(testList,'r');
    %disp(fidList);
    testID = 1;

    %% get each test vectors within the test list
    % test data file name
    %dataFolder_test = fgetl(fidList);    

    %calibration file name
    %dataFolder_calib = fgetl(fidList);

    %module_param_file defines parameters to init each signal processing
    %module
    %module_param_file = fgetl(fidList);

     %parameter file name for the test
    pathGenParaFile = [input_path,'test',num2str(testID),'_param.m'];
    %important to clear the same.m file, since Matlab does not clear cache
    %automatically
    clear(pathGenParaFile);

    %generate parameter file for the test to run
    if PARAM_FILE_GEN_ON == 1     
        parameter_file_gen_json(dataFolder_test, dataFolder_calib, module_param_file, pathGenParaFile, dataPlatform);
    end

    %load calibration parameters
    load(dataFolder_calib)

    % simTopObj is used for top level parameter parsing and data loading and saving
    simTopObj           = simTopCascade('pfile', pathGenParaFile);
    calibrationObj      = calibrationCascade('pfile', pathGenParaFile, 'calibrationfilePath', dataFolder_calib);
    rangeFFTObj         = rangeProcCascade('pfile', pathGenParaFile);
    DopplerFFTObj       = DopplerProcClutterRemove('pfile', pathGenParaFile);
    detectionObj        = CFAR_CASO('pfile', pathGenParaFile);
    DOAObj              = DOACascade('pfile', pathGenParaFile);

    % get system level variables
    platform            = simTopObj.platform;
    numValidFrames      = simTopObj.totNumFrames;
    cnt = 1;
    frameCountGlobal = 0;


   % Get Unique File Idxs in the "dataFolder_test"   
   [fileIdx_unique] = getUniqueFileIdx(dataFolder_test);
   %disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
   %disp((length(fileIdx_unique)));
   %disp(fileIdx_unique);
   
   i_file=1;
   % Get File Names for the Master, Slave1, Slave2, Slave3   
   [fileNameStruct]= getBinFileNames_withIdx(dataFolder_test, fileIdx_unique{i_file});        

   %pass the Data File to the calibration Object
   calibrationObj.binfilePath = fileNameStruct;

   detection_results = [];  

   % Get Valid Number of Frames 
   [numValidFrames dataFileSize] = getValidNumFrames(fullfile(dataFolder_test, fileNameStruct.masterIdxFile));
   %intentionally skip the first frame due to TDA2 

   for frameIdx = 45:45;%numFrames_toRun
        tic
        %read and calibrate raw ADC data            
        calibrationObj.frameIdx = frameIdx;
        frameCountGlobal = frameCountGlobal+1
        %disp("The calibrationObject");
        %disp(calibrationObj);
        adcData = datapath(calibrationObj);
        %disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
        %disp(size(adcData));  % resul size is   256   128    16    12
        % RX Channel re-ordering
        adcData = adcData(:,:,calibrationObj.RxForMIMOProcess,:); 
        % Store the reconstructed data in the appropriate array
        switch i
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%% Create shifted data equations %%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            case 1
                %radar_data_1 = adcData;
                %shifted_data_1= applyRangeShift(adcData,5,15);
                radar_data_1= adcData;
                fprintf("Store one file radar_data1: %d\n", i);
            case 2
                radar_data_2 = adcData;
                fprintf("Store one file radar_data2: %d\n", i);
            case 3
                radar_data_3 = adcData;
                fprintf("Store one file radar_data3: %d\n", i);
            case 4
                radar_data_4 = adcData;
                fprintf("Store one file radar_data4: %d\n", i);
            case 5
                radar_data_5 = adcData;
                fprintf("Store one file radar_data5: %d\n", i);
        end
        adcData=radar_data_1; 
        if i==1
            % Specify the output directory for the wavelet plots
            outputDir = 'D:\FYP-SignalProcessing\DroneSwarmsImages\Wavelet_Plots';

            % Call the function
            plotWaveletForChirps(adcData, outputDir);
            
           
            %disp("The adcData matrix size")
            %disp(size(adcData)); % resul size is   256   128    16    12
            %only take TX and RXs required for MIMO data analysis
            %adcData = adcData
            if mod(frameIdx, 10)==1
                fprintf('Processing %3d frame...\n', frameIdx);
            end
            %perform 2D FFT
            rangeFFTOut = [];
            DopplerFFTOut = [];
            for i_tx = 1: size(adcData,4)   %this size(adcData,4) means number of transmitters 
                % range FFT
                % disp("The size of adcData(:,:,:,i_tx)");
                % disp(size(adcData(:,:,:,i_tx)));
                rangeFFTOut(:,:,:,i_tx)     = datapath(rangeFFTObj, adcData(:,:,:,i_tx));

                DopplerFFTOut(:,:,:,i_tx)   = datapath(DopplerFFTObj, rangeFFTOut(:,:,:,i_tx));
            end
            % CFAR done along only TX and RX used in MIMO array
            %this DopplerFFTOut size is (256,128,16,12)
            DopplerFFTOut = reshape(DopplerFFTOut,size(DopplerFFTOut,1), size(DopplerFFTOut,2), size(DopplerFFTOut,3)*size(DopplerFFTOut,4));
            %disp("After reshaping DopplerFFTout size");
            %disp(size(DopplerFFTOut)); % Shape is (256,128,192)
            %detection
            sig_integrate = 10*log10(sum((abs(DopplerFFTOut)).^2,3) + 1);
            %disp(sig_integrate)            
            detection_results = datapath(detectionObj, DopplerFFTOut);
            detection_results_all{cnt} =  detection_results;
            detect_all_points = [];
            for iobj = 1:length(detection_results)
                detect_all_points (iobj,1)=detection_results(iobj).rangeInd+1;
                detect_all_points (iobj,2)=detection_results(iobj).dopplerInd_org+1;
                detect_all_points (iobj,4)=detection_results(iobj).estSNR;
            end
            if PLOT_ON
                figure(1);
                set(gcf,'units','normalized','outerposition',[0 0 1 1])                
                subplot(2,2,1)               
                plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,size(sig_integrate,2)/2+1),'g','LineWidth',4);hold on; grid on
                for ii=1:size(sig_integrate,2)
                    plot((1:size(sig_integrate,1))*detectionObj.rangeBinSize, sig_integrate(:,ii));hold on; grid on
                    if ~isempty(detection_results)
                        ind = find(detect_all_points(:,2)==ii);
                        if (~isempty(ind))
                            rangeInd = detect_all_points(ind,1);
                            plot(rangeInd*detectionObj.rangeBinSize, sig_integrate(rangeInd,ii),'o','LineWidth',2,...
                                'MarkerEdgeColor','k',...
                                'MarkerFaceColor',[.49 1 .63],...
                                'MarkerSize',6);
                        end
                    end
                end

                %title(['FrameID: ' num2str(cnt)]);
                xlabel('Range(m)');
                ylabel('Receive Power (dB)')
                title(['Range Profile(zero Doppler - thick green line): frameID ' num2str(frameIdx)]);
                hold off;
                subplot(2,2,2);
                %subplot_tight(2,2,2,0.1)
                %disp("The size of the sig_integrate")
                %disp(size(sig_integrate))
                rangeAxis = (1:size(sig_integrate,1)) * detectionObj.rangeBinSize; % Range axis in meters
                velocityAxis = linspace(-20, 20, size(sig_integrate,2)); % Velocity axis in m/s
                imagesc(velocityAxis, rangeAxis, sig_integrate);

                %imagesc((sig_integrate))
                c = colorbar;
                c.Label.String = 'Relative Power(dB)';
                title(' Range/Velocity Plot');
                pause(0.01)
            end
            angles_all_points = [];
            xyz = [];
            %if 0
            if ~isempty(detection_results)
                % DOA, the results include detection results + angle estimation results.
                % access data with angleEst{frame}(objectIdx).fieldName
                angleEst = datapath(DOAObj, detection_results);

                if length(angleEst) > 0
                    for iobj = 1:length(angleEst)
                        angles_all_points (iobj,1:2)=angleEst(iobj).angles(1:2);
                        angles_all_points (iobj,3)=angleEst(iobj).estSNR;
                        angles_all_points (iobj,4)=angleEst(iobj).rangeInd;
                        angles_all_points (iobj,5)=angleEst(iobj).doppler_corr;
                        angles_all_points (iobj,6)=angleEst(iobj).range;
                        %switch left and right, the azimuth angle is flipped
                        xyz(iobj,1) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,1)*-1)*cosd(angles_all_points (iobj,2));
                        xyz(iobj,2) = angles_all_points (iobj,6)*cosd(angles_all_points (iobj,1)*-1)*cosd(angles_all_points (iobj,2));
                        %switch upside and down, the elevation angle is flipped
                        xyz(iobj,3) = angles_all_points (iobj,6)*sind(angles_all_points (iobj,2)*-1);
                        xyz(iobj,4) = angleEst(iobj).doppler_corr;
                        xyz(iobj,9) = angleEst(iobj).dopplerInd_org;
                        xyz(iobj,5) = angleEst(iobj).range;
                        xyz(iobj,6) = angleEst(iobj).estSNR;
                        xyz(iobj,7) = angleEst(iobj).doppler_corr_overlap;
                        xyz(iobj,8) = angleEst(iobj).doppler_corr_FFT;

                    end
                    angles_all_all{cnt} = angles_all_points;
                    xyz_all{cnt}  = xyz;
                    maxRangeShow = detectionObj.rangeBinSize*rangeFFTObj.rangeFFTSize;
                    %tic
                    if PLOT_ON
                        moveID = find(abs(xyz(:,4))>=0);
                        subplot(2,2,4);                        

                        if cnt==1
                            scatter3(xyz(moveID,1),xyz(moveID,2),xyz(moveID,3),45,(xyz(moveID,4)),'filled');
                        else
                            yz = [xyz_all{cnt}; xyz_all{cnt-1}];
                            scatter3(xyz(moveID,1),xyz(moveID,2),xyz(moveID,3),45,(xyz(moveID,4)),'filled');
                        end

                        c = colorbar;
                        c.Label.String = 'velocity (m/s)';                        
                        grid on;

                        xlim([-20 20])
                        ylim([1 maxRangeShow])
                        %zlim([-4 4])
                        zlim([-5 5])
                        xlabel('X (m)')
                        ylabel('y (m)')
                        zlabel('Z (m)')                        

                        view([-9 15])                        
                        title(' 3D point cloud');

                        %plot range and azimuth heatmap
                        subplot(2,2,3)
                        STATIC_ONLY = 1;
                        minRangeBinKeep =  5;
                        rightRangeBinDiscard =  20;
                        [mag_data_static(:,:,frameCountGlobal) mag_data_dynamic(:,:,frameCountGlobal) y_axis x_axis]= plot_range_azimuth_2D(detectionObj.rangeBinSize, DopplerFFTOut,...
                            length(calibrationObj.IdTxForMIMOProcess),length(calibrationObj.RxForMIMOProcess), ...
                            detectionObj.antenna_azimuthonly, LOG_ON, STATIC_ONLY, PLOT_ON, minRangeBinKeep, rightRangeBinDiscard);
                        title('range/azimuth heat map static objects')
                if (DISPLAY_RANGE_AZIMUTH_DYNAMIC_HEATMAP)                   
                figure(2)
                subplot(121);
                surf(y_axis, x_axis, (mag_data_static(:,:,frameCountGlobal)).^0.4,'EdgeColor','none');
                view(2);
                xlabel('meters');    ylabel('meters')
                title({'Static Range-Azimuth Heatmap',strcat('Current Frame Number = ', num2str(frameCountGlobal))})

                subplot(122);
                surf(y_axis, x_axis, (mag_data_dynamic(:,:,frameCountGlobal)).^0.4,'EdgeColor','none');
                view(2);    
                xlabel('meters');    ylabel('meters')
                title('Dynamic HeatMap')
                end
                pause(0.1) 


                    end

                end

            end

            cnt = cnt + 1;    
       toc
        end
   end


   

    ind = strfind(dataFolder_test, '\');
    testName = dataFolder_test(ind(end-1)+1:(ind(end)-1));
    if SAVEOUTPUT_ON == 1
        % Define the output directory
        outputDir = '.\main\cascade\output\';

        % Check if the directory exists, and create it if it doesn't
        if ~exist(outputDir, 'dir')
            mkdir(outputDir);
        end

        % Save the output file
        save([outputDir, 'newOutput_', testName, '.mat'], 'angles_all_all', 'detection_results_all', 'xyz_all');
    end

    testID = testID + 1;


end
function shiftedData = applyRangeShift(adcData, delta_r, Rmax)
    % APPLYRANGESHIFT Applies a range shift to a radar data cube.
    % Inputs:
    %   adcData - The original radar data cube (Range x Chirps x Antennas)
    %   delta_r - Range shift in meters
    %   Rmax - Maximum range of the radar system
    %   sample_rate - ADC sample rate in Hz
    % Output:
    %   shiftedData - The radar data cube after applying the range shift

    % Reshape adcData to 3D if it is 4D
    adcData = reshape(adcData, size(adcData, 1), size(adcData, 2), size(adcData, 3) * size(adcData, 4));

    % Dimensions of the radar data cube
    [N, C, A] = size(adcData); % N: Range, C: Chirps, A: Antennas

    % Calculate the phase shift for each range sample
    range_shift = delta_r; % Range shift in meters
    
    slope = 79e12; % Chirp slope in Hz/s
    c = 3e8; % Speed of light in m/s

    % Step 2: Simulate radar data
    % Assuming radar_data is a 2D matrix with dimensions (n_chirps, n_samples)
    
    n_samples = 256; % Number of range bins (ADC samples)


    % Step 3: Calculate sample rate
    sample_rate = 8000000; % Sample rate (Hz)

    % Step 4: Generate time axis for ADC samples
    time = (0:n_samples-1) / sample_rate; % Time vector for range samples

    % Step 5: Compute frequency shift due to range shift
    frequency_shift = 2 * slope * range_shift / c; % Frequency shift due to range displacement
    shiftedData = adcData;
    % Step 6: Calculate phase shift for each range sample
    phase_shift = 2 * pi * frequency_shift * time; % Phase shift for each ADC sample
    % Apply the range shift across the range dimension
    for chirpIdx = 1:C % Loop over chirps
        for rangeIdx = 1:N % Loop over range samples
            for antennaIdx = 1:A % Loop over antennas
                % Apply the range shift to the current range sample
                shiftedData(rangeIdx, chirpIdx, antennaIdx) = ...
                    adcData(rangeIdx, chirpIdx, antennaIdx) * ...
                    exp(1i * phase_shift(rangeIdx));
            end
        end
    end

    % Reshape adcData back to 4D if originally 4D
    adcData = reshape(shiftedData, size(shiftedData, 1), size(shiftedData, 2), size(shiftedData, 3) / 12, 12);

    shiftedData = adcData; % Assign back to shiftedData for consistent output

    % Display a confirmation message
    disp('Range shift applied successfully to the radar data cube.');
end
function plotWaveletForChirps(adcData, outputDir)
    % Function to compute and plot wavelet transform for all chirps in groups of 16
    % Inputs:
    % - adcData: 3D matrix (size: [samples, chirps, antennas])
    % - outputDir: Directory to save the wavelet plots

    % Ensure output directory exists
    adcData = reshape(adcData,size(adcData,1), size(adcData,2), size(adcData,3)*size(adcData,4));%make 3D

    if ~exist(outputDir, 'dir')
        mkdir(outputDir);
    end

    % Define sampling frequency
    Fs = 8e6; % 8 MHz

    % Total number of chirps
    totalChirps = size(adcData, 2);

    % Loop over chirps in groups of 16
    chirpsPerGroup = 16;
    numGroups = ceil(totalChirps / chirpsPerGroup);

    for groupIdx = 1:numGroups
        % Determine chirps in the current group
        startChirp = (groupIdx - 1) * chirpsPerGroup + 1;
        endChirp = min(groupIdx * chirpsPerGroup, totalChirps);

        
            % Select data for the current chirp
            selectedData = adcData(:, startChirp:endChirp, 1); % Assuming antenna index 1

            % Flatten the data and convert to complex double
            flattenedData = complex(double(real(selectedData(:))), double(imag(selectedData(:))));

            % Separate into real and imaginary parts (optional saving)
            realPart = real(flattenedData);
            imagPart = imag(flattenedData);

            % Save real and imaginary parts (optional)
            %realFile = fullfile(outputDir, sprintf('real_part_chirp%d.mat', groupIdx));
            %imagFile = fullfile(outputDir, sprintf('imag_part_chirp%d.mat', chirpIdx));
            %save(realFile, 'realPart');
            %save(imagFile, 'imagPart');

            % Define time vector
            t = (0:length(flattenedData) - 1) / Fs;

            % Perform Continuous Wavelet Transform (CWT) for real part
            [cfs_real, f_real] = cwt(realPart, Fs);

            % Plot wavelet transform for real part
            %{
            figure;
            % Contour plot for real part
            subplot(2, 1, 1);
            contour(t, f_real, abs(cfs_real).^2);
            axis tight;
            grid on;
            xlabel('Time (s)');
            ylabel('Frequency (Hz)');
            title(sprintf('CWT Contour Plot (Real Part) - Chirp %d', groupIdx));
            ylim([0 1.6e6]);
            xlim([0 2e-4]);
            %}
            % Pcolor plot for real part
            figure;
            h_real = pcolor(t, f_real, abs(cfs_real).^2);
            set(h_real, 'EdgeColor', 'none');
            colormap jet;
            colorbar;
            xlabel('Time (s)');
            ylabel('Frequency (Hz)');
            title(sprintf('CWT Pcolor Plot (Real Part) - Chirpgroup %d', groupIdx));
            ylim([0 1.6e6]);
            xlim([0 2e-4]);
            
            hold on;
            contour(t, f_real, abs(cfs_real).^2, 'LineWidth', 1, 'LineColor', 'k'); % Add contour lines with black edges
            hold off;

            % Save the real part plot
            saveas(gcf, fullfile(outputDir, sprintf('phantomchirpgroup%d_wavelet_real.png', groupIdx)));
            close(gcf);

            % Perform Continuous Wavelet Transform (CWT) for imaginary part
            [cfs_imag, f_imag] = cwt(imagPart, Fs);

            % Plot wavelet transform for imaginary part
            %{
            figure;
            % Contour plot for imaginary part
            subplot(2, 1, 1);
            contour(t, f_imag, abs(cfs_imag).^2);
            axis tight;
            grid on;
            xlabel('Time (s)');
            ylabel('Frequency (Hz)');
            title(sprintf('CWT Contour Plot (Imaginary Part) - Chirp %d', groupIdx));
            ylim([0 1.6e6]);
            xlim([0 2e-4]);
            %}
            
            % Pcolor plot for imaginary part
            figure;
            h_imag = pcolor(t, f_imag, abs(cfs_imag).^2);
            set(h_imag, 'EdgeColor', 'none');
            colormap jet;
            colorbar;
            xlabel('Time (s)');
            ylabel('Frequency (Hz)');
            title(sprintf('CWT Pcolor Plot (Imaginary Part) - Chirp %d', groupIdx));
            ylim([0 1.6e6]);
            xlim([0 2e-4]);
            % Overlay contours on the pcolor plot for enhanced visualization
            hold on;
            contour(t, f_imag, abs(cfs_imag).^2, 'LineWidth', 1, 'LineColor', 'k'); % Add contour lines with black edges
            hold off;

            % Save the imaginary part plot
            saveas(gcf, fullfile(outputDir, sprintf('phantomchirp%d_wavelet_imag.png', groupIdx)));
            close(gcf);
    end

    disp('Wavelet plots for all chirps have been generated and saved.');
end

