
dataInput = readmatrix('D:\drone_swarms_stationary\four_drones\drone_swarm1.csv');
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
% Determine sizes
realCols = size(dataInput, 2) / 2; % Split columns between real and imaginary parts
rowCount = 256 * 128;
colCount = 16 * 12;

% Separate real and imaginary parts
realComponent = dataInput(:, 1:realCols);
imagComponent = dataInput(:, realCols + 1:end);

% Reconstruct the 2D complex array
complexData = complex(realComponent, imagComponent);

% Reshape back to the original 4D array
data4D = reshape(complexData, [256, 128, 16, 12]);
radarDataChain = data4D;
disp('4D array reconstructed from complex_data_2D.csv');
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

freqVector = linspace(-pulseRepetitionFrequency / 2, pulseRepetitionFrequency / 2, nfft); % Frequency Axis
% Corrected MATLAB code
% Calculate time axis range correctly
num_time_bins = size(specgramAbs, 2); % Number of time bins in spectrogram
disp(size(num_time_bins));
total_duration = (length(signalData) / sampleRate); % Total signal duration in seconds
time_axis_range = linspace(0, total_duration, num_time_bins); % Time axis from 0 to total duration
disp(size(time_axis_range));
timeAxis = (0:1)*total_duration; % Time

% Plot the spectrogram
figureHandle = figure(3); % Specify figure number
set(figureHandle, 'Visible', 'on'); % Make the figure visible
colormap(jet(256)); % Set colormap

imagesc(timeAxis, freqVector, ...
        20 * log10(abs(specgramAbs / max(specgramAbs(:)))));
xlabel('Time (sec)');
ylabel('Frequency (Hz)');

% Add colorbar and set its label
colorBarHandle = colorbar;
colorBarHandle.Label.String = 'Power (dB)';

% Set color axis limits and adjust plot orientation
caxis([-30 0]); % Set color axis limits
set(gca, 'YDir', 'normal'); % Normal y-axis direction
set(gcf, 'Color', 'w'); % Set figure background color to white
