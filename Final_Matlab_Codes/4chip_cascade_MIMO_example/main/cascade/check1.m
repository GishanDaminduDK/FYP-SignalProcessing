filePath = 'D:\Drone-Swarm-Detection-with-AWR2243\Our data\chirps_signal_1.csv'; % Update with your actual file path

% Read the .csv file (assuming complex numbers are in 'a+bi' format)
data = readmatrix(filePath, 'Delimiter', ',');

% Get the number of chirps and samples
numChirps = 128; % Total number of chirps
numSamples = 256; % Samples per chirp
% Reshape the data into a 128x256 matrix
radarData = complex(data(:, 1:numSamples));  %This is a matrix radarData diamenstion is, 128*256
numSamplePerChirp = 256;    % Number of samples per chirp
numChirpPerLoop = 12;       % Number of chirps per loop
numLoops = 128;              % Number of loops per frame
numRXPerDevice = 4;         % Number of receiving channels per device
numDevices = 4;             % Number of devices in the cascade (adjust based on your setup)
% Constants (adjust based on your radar parameters)
fc = 77e9;                  % Radar operating frequency (77 GHz for mmWave radar)
c = 3e8;                    % Speed of light (m/s)
sweepBandwidth =3.16e9;     % Bandwidth of the FMCW radar sweep (3.16 GHz)
chirpDuration = 40e-6;       % Chirp duration (40 microseconds)
  
radarData=transpose(radarData);
disp("The size of the radarData matrix");
disp(size(radarData));
Nfft_doppler = 2^(ceil(log2(numLoops)));
Nfft_range = 2^(ceil(log2(numSamplePerChirp)));
range_fft = fft(radarData, Nfft_range, 1); % Range FFT
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
doppler_fft = doppler_fft * velocity_Resolution;
magnitude_array = 20 * log10(abs(doppler_fft));
% Plot results
figure;
imagesc(velocity_axis, range_axis, magnitude_array);
xlabel('Velocity (m/s)');
ylabel('Range (m)');
title('Original Range-Velocity Map');
colorbar;
axis xy;



