
data_combined = readmatrix('D:\Drone-Swarm-Detection-with-AWR2243\Our data\complex_data_2D.csv');

% Determine sizes
num_real_cols = size(data_combined, 2) / 2; % Split columns between real and imaginary parts
rows = 256 * 128;
cols = 16 * 12;

% Separate real and imaginary parts
real_part = data_combined(:, 1:num_real_cols);
imag_part = data_combined(:, num_real_cols+1:end);

% Reconstruct the 2D complex array
data_reshaped = complex(real_part, imag_part);

% Reshape back to the original 4D array
data_reconstructed = reshape(data_reshaped, [256, 128, 16, 12]);
radar_data_Rxchain = data_reconstructed ;
disp('Test.csv');
disp(size(radar_data_Rxchain));

